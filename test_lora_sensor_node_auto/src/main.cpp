// [File: alynxneko/greenhouse-esp/Greenhouse-ESP-ce21ed56fcbaa14510252b1a6bbbb5005a9de09e/test_lora_sensor_node_auto/src/main.cpp]
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
unsigned long lastRotationTime = 0; // For tracking rotation intervals
bool autoSendEnabled = true;

// --- Stepper Sequence (Half-step) ---
const uint8_t seq[8][4] = {
  {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
  {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}
};

// ------------------ DRYING MODEL PARAMETERS ------------------
// k value for Lewis model (approximate for coffee drying, adjust based on experiments)
const float k_drying = 0.0025; 
const int RACK_COUNT = 8;
int currentRack = 1;            
float rackM[RACK_COUNT];        
float rack_k_multiplier[RACK_COUNT]; 
enum OpMode { AUTO = 0, SEMI = 1, NOTIFY = 2 };
OpMode currentMode = AUTO; 
float initialMoisturePercent = 15.0; // Default initial
const float M_TARGET = 12.0;

// Temp Thresholds for Logic
const float TEMP_HOT = 30.0;
const float TEMP_COOL = 25.0;

const int STEPS_PER_REV = 4096; 
long currentStepPosition = 0;

// GAB Model Parameters for EMC Calculation
struct GabParams { float M0; float C; float K; float T; };
const GabParams gabTable[] = {
  // { M0,   C,    K,    T }
  {5.50, 20.0, 0.88, 25.0}, // At 25°C
  {5.00, 18.0, 0.86, 32.0}, // At 32°C
  {4.50, 15.0, 0.84, 39.0}  // At 39°C
};

// ------------------ FUNCTION DECLARATIONS ------------------

void initRackMultipliers() {
  rack_k_multiplier[0] = 1.00; 
  rack_k_multiplier[1] = 0.95;
  rack_k_multiplier[2] = 0.90;
  rack_k_multiplier[3] = 0.85;
  rack_k_multiplier[4] = 0.85;
  rack_k_multiplier[5] = 0.90;
  rack_k_multiplier[6] = 0.95;
  rack_k_multiplier[7] = 1.00; 
}

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

float calculateEMC(float T, float RH) {
  float a_w = RH / 100.0f;
  if (a_w <= 0.0f) return 0.0f;
  if (a_w >= 0.99f) a_w = 0.99f; // Prevent singularity
  GabParams p = interpGabParams(T);
  float denom = (1.0f - p.K * a_w);
  if (denom == 0.0f) denom = 1e-6;
  float M = (p.M0 * p.C * p.K * a_w) / (denom * (denom + p.C * p.K * a_w));
  return M;
}

void updateRackMoistures(float Me, float dt_min) {
  for (int i = 0; i < RACK_COUNT; i++) {
    float k_eff = k_drying * rack_k_multiplier[i]; 
    // Lewis Model: M(t) = Me + (M0 - Me) * exp(-kt)
    // Iterative: M_new = Me + (M_old - Me) * exp(-k * dt)
    float decay = expf(-k_eff * dt_min);
    float M_old = rackM[i];
    float M_new = Me + (M_old - Me) * decay;
    rackM[i] = M_new;
  }
}

// Formula 3.5 from Thesis: t* = -1/k * ln( (M_target - Me) / (M_current - Me) )
int calculatePredTime(float M_current, float Me) {
  if (M_current <= M_TARGET) return 0; // Already dry
  if (Me >= M_TARGET) return 999; // Cannot dry to target with current air (EMC too high)
  
  float numerator = M_TARGET - Me;
  float denominator = M_current - Me;
  
  if (denominator <= 0) return 999; // Should not happen if checks above pass

  float ratio = numerator / denominator;
  if (ratio <= 0) return 0; // Safety

  float t_min = (-1.0f / k_drying) * log(ratio);
  return (int)t_min;
}

// --- Stepper Logic ---
void stepMotor(int stepIndex) {
  digitalWrite(in1, seq[stepIndex][0]);
  digitalWrite(in2, seq[stepIndex][1]);
  digitalWrite(in3, seq[stepIndex][2]);
  digitalWrite(in4, seq[stepIndex][3]);
}
// --- HELPER: Normalize Angle ---
float getAngle() {
  // Convert steps to 0-360 degrees
  long pos = currentStepPosition % STEPS_PER_REV;
  if (pos < 0) pos += STEPS_PER_REV;
  return (pos * 360.0f) / STEPS_PER_REV;
}

// --- UPDATED: Stepper Logic ---
void rotateStepper(int steps, bool clockwise = true) {
  if (clockwise) {
    for (int i = 0; i < steps; i++) {
      stepMotor(i % 8);
      delay(stepDelay);
    }
    currentStepPosition += steps; // Track Position
  } else {
    for (int i = steps - 1; i >= 0; i--) {
      stepMotor(i % 8);
      delay(stepDelay);
    }
    currentStepPosition -= steps; // Track Position
  }
  
  // Release coils
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

void shiftExposureMultipliers(bool clockwise=true) {
  // Simplified shift for visualization logic
  if (clockwise) {
    float last = rack_k_multiplier[RACK_COUNT-1];
    for (int i = RACK_COUNT-1; i>0; i--) rack_k_multiplier[i] = rack_k_multiplier[i-1];
    rack_k_multiplier[0] = last;
    currentRack = (currentRack % RACK_COUNT) + 1; 
  }
}

// --- NEW AUTOMATIC LOGIC ---
void processAutoLogic(float T) {
  if (currentMode != AUTO) return;

  unsigned long now = millis();
  unsigned long interval = 0;

  // 1. Determine interval based on Temperature
  if (T > TEMP_HOT) {
    // HOT (>30): Fast rotation (e.g., every 30 secs)
    interval = 30 * 1000UL; 
  } else if (T >= TEMP_COOL) {
    // AVERAGE (25-30): Medium rotation (e.g., every 2 mins)
    interval = 2 * 60 * 1000UL;
  } else {
    // COOL (<25): Slow rotation (e.g., every 10 mins)
    interval = 10 * 60 * 1000UL;
  }

  // 2. Check time
  if (now - lastRotationTime > interval) {
    lastRotationTime = now;
    Serial.println("Auto Logic: Rotating 180 deg...");
    // 180 degrees = approx 2048 steps (assuming 4096 per rev)
    rotateStepper(2048, true); 
    shiftExposureMultipliers(true); 
    shiftExposureMultipliers(true); // Shift logic twice since we moved 180 deg? (depends on logic mapping)
    // Actually just shift once per significant move for simplicity or align with steps
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  
  pinMode(FAN_PIN, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.receive();

  if (!bme.begin(0x76, &Wire)) {
    Serial.println("BME init failed!");
    while (1);
  }

  initRackMultipliers();
  for (int i = 0; i < RACK_COUNT; i++) {
      rackM[i] = initialMoisturePercent;
  }
  Serial.println("Sensor Node Ready.");
}

// --- Communications ---
void sendStatusPacket() {
  float T = bme.readTemperature();
  float H = bme.readHumidity();
  float emc = calculateEMC(T, H);
  bool fanStatus = digitalRead(FAN_PIN);
  float currentAngle = getAngle();

  // Average Moisture
  float avgM = 0;
  for(float m : rackM) avgM += m;
  avgM /= RACK_COUNT;

  // New Prediction Calculation
  int predMins = calculatePredTime(avgM, emc);

  String payload = "STATUS:T=" + String(T, 2);
  payload += ";H=" + String(H, 2);
  payload += ";EMC=" + String(emc, 2);
  payload += ";RACK=" + String(currentRack);
  payload += ";ANG=" + String(currentAngle, 1);
  payload += ";FAN=" + String(fanStatus ? "1" : "0");
  payload += ";PRED=" + String(predMins);
  payload += ";MODE=" + String(currentMode);
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
  else if (cmd == "PAUSEAUTO") {
    autoSendEnabled = false;
    // Optionally send a confirmation
    // LoRa.beginPacket(); LoRa.print("AUTO_PAUSED"); LoRa.endPacket(); LoRa.receive();
  }
  else if (cmd == "RESUMEAUTO") {
    autoSendEnabled = true;
  }
  else if (cmd == "FANON") {
    digitalWrite(FAN_PIN, HIGH);
    sendStatusPacket(); 
  } 
  else if (cmd == "FANOFF") {
    digitalWrite(FAN_PIN, LOW);
    sendStatusPacket();
  }
  else if (cmd == "NEXT") {
    rotateStepper(512, true); // 45 deg
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

  // 2. Periodic Logic
  unsigned long now = millis();
  
  // Read Environment continuously for Auto Logic
  float T = bme.readTemperature();
  float H = bme.readHumidity();
  
  // Execute Auto Rotation Logic
  processAutoLogic(T);

  // Fan Logic (Hysteresis)
  if (currentMode == AUTO) {
      if (H > 75.0f) digitalWrite(FAN_PIN, HIGH);
      else if (H < 50.0f) digitalWrite(FAN_PIN, LOW);
  }

  // Periodic Update & Model Calc
  if (autoSendEnabled && (now - lastSend) >= SEND_INTERVAL) {
    lastSend = now;
    float emc = calculateEMC(T, H);
    float dt_min = (float)SEND_INTERVAL / 60000.0f;
    updateRackMoistures(emc, dt_min);
    sendStatusPacket(); 
    Serial.printf("T:%.2f H:%.2f EMC:%.2f Pred:%d\n", T, H, emc, calculatePredTime(initialMoisturePercent, emc));
  }
}