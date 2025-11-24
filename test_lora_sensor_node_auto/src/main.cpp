#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

// --- Hardware Pins ---
#define LORA_CS   5
#define LORA_RST  14
#define LORA_DIO0 26
#define FAN_PIN   33 

// Stepper Pins (28BYJ-48 + ULN2003)
const int in1 = 13;
const int in2 = 27;
const int in3 = 25;
const int in4 = 32;
const int stepDelay = 3; // ms between half-steps

// --- BME280 Settings ---
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

// --- Timing ---
#define SEND_INTERVAL (10 * 1000UL)  // 10 seconds update interval
unsigned long lastSend = 0;

// --- Stepper Sequence (Half-step) ---
const uint8_t seq[8][4] = {
  {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
  {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}
};

// ------------------ DRYING MODEL PARAMETERS ------------------
const float k_base_min = 0.003; // Drying rate constant (per minute)
const int RACK_COUNT = 8;
int currentRack = 1;            // Current top rack (1-8)
float rackM[RACK_COUNT];        // Current estimated moisture (% dry basis)
float rack_k_multiplier[RACK_COUNT]; // Relative exposure multipliers
enum OpMode { AUTO = 0, SEMI = 1, NOTIFY = 2 };
OpMode currentMode = AUTO; // Default Automatic
// Ganti initialMoisturePercent agar bisa diubah
float initialMoisturePercent = 13.0;
const float M_TARGET = 12.0;

// GAB Model Parameters for EMC Calculation
struct GabParams { float M0; float C; float K; float T; };
const GabParams gabTable[] = {
  {3.98, 0.94, 0.87, 25.0},
  {3.55, 0.95, 0.89, 32.0},
  {3.26, 0.97, 0.91, 39.0}
};

// ------------------ FUNCTION DECLARATIONS ------------------

void initRackMultipliers() {
  rack_k_multiplier[0] = 1.00; // Rack 1 (Top) - Best drying
  rack_k_multiplier[1] = 0.95;
  rack_k_multiplier[2] = 0.90;
  rack_k_multiplier[3] = 0.85;
  rack_k_multiplier[4] = 0.85;
  rack_k_multiplier[5] = 0.90;
  rack_k_multiplier[6] = 0.95;
  rack_k_multiplier[7] = 1.00; // Rack 8 (Bottom) - Also good
}

// Interpolate GAB parameters based on Temperature
GabParams interpGabParams(float T) {
  if (T <= gabTable[0].T) return gabTable[0];
  if (T >= gabTable[2].T) return gabTable[2];
  for (int i=0; i<2; i++) {
    if (T >= gabTable[i].T && T <= gabTable[i+1].T) {
      float t0 = gabTable[i].T;
      float t1 = gabTable[i+1].T;
      float f = (T - t0) / (t1 - t0);
      GabParams out;
      out.T = T;
      out.M0 = gabTable[i].M0 + f * (gabTable[i+1].M0 - gabTable[i].M0);
      out.C  = gabTable[i].C  + f * (gabTable[i+1].C  - gabTable[i].C);
      out.K  = gabTable[i].K  + f * (gabTable[i+1].K  - gabTable[i].K);
      return out;
    }
  }
  return gabTable[1];
}

// Calculate Equilibrium Moisture Content (EMC)
float calculateEMC(float T, float RH) {
  float a_w = RH / 100.0f;
  if (a_w <= 0.0f) return 0.0f;
  GabParams p = interpGabParams(T);
  float denom = (1.0f - p.K * a_w);
  if (denom == 0.0f) denom = 1e-6;
  float M = (p.M0 * p.C * p.K * a_w) / (denom * (denom + p.C * p.K * a_w));
  return M;
}

// Update moisture of all racks based on EMC and time elapsed
void updateRackMoistures(float Me, float dt_min) {
  for (int i = 0; i < RACK_COUNT; i++) {
    float k_eff = k_base_min * rack_k_multiplier[i]; // effective rate per minute
    float decay = expf(-k_eff * dt_min);
    float M_old = rackM[i];
    float M_new = Me + (M_old - Me) * decay;
    rackM[i] = M_new;
  }
}

// --- Stepper Logic ---
void stepMotor(int stepIndex) {
  digitalWrite(in1, seq[stepIndex][0]);
  digitalWrite(in2, seq[stepIndex][1]);
  digitalWrite(in3, seq[stepIndex][2]);
  digitalWrite(in4, seq[stepIndex][3]);
}

void rotateStepper(int steps, bool clockwise = true) {
  if (clockwise) {
    for (int i = 0; i < steps; i++) {
      stepMotor(i % 8);
      delay(stepDelay);
    }
  } else {
    for (int i = steps - 1; i >= 0; i--) {
      stepMotor(i % 8);
      delay(stepDelay);
    }
  }
  // Release coils
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

// Shift logic array when physical rack rotates
void shiftExposureMultipliers(bool clockwise=true) {
  if (clockwise) {
    float last = rack_k_multiplier[RACK_COUNT-1];
    for (int i = RACK_COUNT-1; i>0; i--) rack_k_multiplier[i] = rack_k_multiplier[i-1];
    rack_k_multiplier[0] = last;
    currentRack = (currentRack % RACK_COUNT) + 1; 
  } else {
    float first = rack_k_multiplier[0];
    for (int i=0;i<RACK_COUNT-1;i++) rack_k_multiplier[i] = rack_k_multiplier[i+1];
    rack_k_multiplier[RACK_COUNT-1] = first;
    currentRack = (currentRack - 2 + RACK_COUNT) % RACK_COUNT + 1; 
  }
}

// Check if drying is uneven and rotate if necessary
bool maybeRotateAndActuate() {
  // Jika mode NOTIFY, tidak melakukan apa-apa
  if (currentMode == NOTIFY) return false;

  float maxM = rackM[0], minM = rackM[0];
  for (int i=0; i<RACK_COUNT; i++) { 
    maxM = max(maxM, rackM[i]); 
    minM = min(minM, rackM[i]); 
  }
  float diff = maxM - minM;

  bool needRotation = (diff > 1.0f || maxM > (M_TARGET + 1.0f));

  // Jika mode SEMI, kita hanya kirim status "Need Rotation" tapi tidak gerak otomatis
  // (Akan ditangani logic pengiriman packet, di sini return false agar motor diam)
  if (currentMode == SEMI) {
     return false; 
  }

  // Jika mode AUTO, lakukan aksi fisik
  if (needRotation) {
    rotateStepper(512, true);
    shiftExposureMultipliers(true);
    return true;
  }
  return false;
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  
  // Hardware Init
  pinMode(FAN_PIN, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  
  // LoRa Init
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(10);    // Was missing
  LoRa.setSignalBandwidth(125E3); // Was missing
  LoRa.setCodingRate4(5);         // Was missing
  LoRa.enableCrc();               // Was missing
  LoRa.receive();

  // BME280 Init
  // Uses default Wire pins (SDA=21, SCL=22) and Address 0x76
  if (!bme.begin(0x76, &Wire)) {
    Serial.println("BME init failed!");
    while (1);
  }

  // Model Init
  initRackMultipliers();
  for (int i = 0; i < RACK_COUNT; i++) {
      rackM[i] = initialMoisturePercent;
  }
  Serial.println("Sensor Node Ready.");
}

// --- Communications ---

// Calculates EMC based on current sensor readings and sends packet
void sendStatusPacket() {
  // ... baca sensor ...
  float T = bme.readTemperature();
  float H = bme.readHumidity();
  float emc = calculateEMC(T, H);
  bool fanStatus = digitalRead(FAN_PIN);

  // Prediksi waktu kering (simple estimation)
  float avgM = 0;
  for(float m : rackM) avgM += m;
  avgM /= RACK_COUNT;
  int predMins = (avgM > M_TARGET) ? (int)((avgM - M_TARGET) / (k_base_min * 100)) : 0; 

  // --- FIX: Membangun String dengan aman ---
  String payload = "STATUS:T=" + String(T, 2);
  payload += ";H=" + String(H, 2);
  payload += ";EMC=" + String(emc, 2);
  payload += ";RACK=" + String(currentRack);
  
  // Perbaikan di sini: Pisahkan penambahan string atau gunakan String()
  payload += ";FAN=";
  payload += (fanStatus ? "1" : "0");
  
  payload += ";PRED=" + String(predMins);
  payload += ";MODE=" + String(currentMode);

  // Append data moisture tiap rak
  payload += ";M_DATA="; 
  for (int i = 0; i < RACK_COUNT; i++) {
    payload += String(rackM[i], 2);
    if (i < RACK_COUNT - 1) payload += ",";
  }

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();
  LoRa.receive();
}

void processCommand(String cmd) {
  cmd.toUpperCase();
  
  if (cmd == "REQSTATUS") {
    sendStatusPacket();
  } 
  // Ganti logic FAN agar menghormati mode jika perlu, 
  // tapi biasanya manual override (tombol app) tetap jalan di semua mode.
  else if (cmd == "FANON") {
    digitalWrite(FAN_PIN, HIGH);
    sendStatusPacket(); 
  } 
  else if (cmd == "FANOFF") {
    digitalWrite(FAN_PIN, LOW);
    sendStatusPacket();
  }
  else if (cmd == "NEXT") {
    // Manual step
    rotateStepper(512, true);
    shiftExposureMultipliers(true);
    sendStatusPacket();
  }
  else if (cmd.startsWith("STEP:")) {
    int angle = cmd.substring(5).toInt();
    bool cw = angle >= 0;
    int steps = abs(angle) * (512.0 / 45.0);
    rotateStepper(steps, cw);
    sendStatusPacket(); 
  }
  // --- COMMAND BARU ---
  else if (cmd.startsWith("SETMODE:")) {
    int m = cmd.substring(8).toInt();
    if (m >= 0 && m <= 2) {
      currentMode = (OpMode)m;
      sendStatusPacket();
    }
  }
  else if (cmd.startsWith("SETMOIST:")) {
    float m = cmd.substring(9).toFloat();
    if (m > 0) {
      initialMoisturePercent = m;
      // Reset current calculation logic
      for (int i = 0; i < RACK_COUNT; i++) rackM[i] = m;
      sendStatusPacket();
    }
  }
}

// --- MAIN LOOP ---
void loop() {
  // 1. Check for LoRa commands
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String msg = "";
    while (LoRa.available()) msg += (char)LoRa.read();
    msg.trim();
    
    if (msg.length() > 0) {
      Serial.println("RX: " + msg);
      processCommand(msg);
    }
  }

  // 2. Periodic Update (Drying Logic)
  unsigned long now = millis();
  if ((now - lastSend) >= SEND_INTERVAL) {
    lastSend = now;

    // Read Environment
    float T = bme.readTemperature();
    float H = bme.readHumidity();
    float emc = calculateEMC(T, H);
    
    // Calculate elapsed time in minutes
    float dt_min = (float)SEND_INTERVAL / 60000.0f;

    // Update Drying Model
    updateRackMoistures(emc, dt_min);

    // Actuation Logic
    bool rotated = maybeRotateAndActuate();
    
    // Fan Logic (Simple hysteresis)
    if (H > 75.0f) digitalWrite(FAN_PIN, HIGH);
    else if (H < 40.0f) digitalWrite(FAN_PIN, LOW);

    // Send Update to App (INCLUDES RACK DATA NOW)
    sendStatusPacket(); 
    
    // Debug Info
    Serial.printf("T:%.2f H:%.2f EMC:%.2f Rotated:%d\n", T, H, emc, rotated);
  }
}