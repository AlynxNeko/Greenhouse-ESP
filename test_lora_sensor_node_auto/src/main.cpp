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

// --- Fan Logic Configuration ---
#define FAN_ON    HIGH
#define FAN_OFF   LOW

// Stepper Pins (28BYJ-48 + ULN2003)
const int in1 = 13;
const int in2 = 27;
const int in3 = 25;
const int in4 = 32;
const int stepDelay = 3; 

// --- BME280 Settings ---
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

// --- Timing ---
#define SEND_INTERVAL (10 * 1000UL) 
unsigned long lastSend = 0;
unsigned long lastRotationTime = 0; 
bool autoSendEnabled = true;

// --- Stepper Sequence ---
const uint8_t seq[8][4] = {
  {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
  {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}
};

// ------------------ LOGIC & STATE ------------------
// [FIX 2] Slower drying constant for realistic sun drying
const float k_drying = 0.0003; 

const int RACK_COUNT = 8;
int currentRack = 1;            
float rackM[RACK_COUNT];        
float rack_k_multiplier[RACK_COUNT]; 

// Modes: 
// 0: AUTO (Auto Act)
// 1: SEMI (Manual Act + Notify)
// 2: NOTIFY (Manual Act + Notify)
enum OpMode { AUTO = 0, SEMI = 1, NOTIFY = 2 };
OpMode currentMode = AUTO; 

float initialMoisturePercent = 15.0; 
const float M_TARGET = 12.0;

// Thresholds for Rotation
const float TEMP_HOT = 30.0;
const float TEMP_COOL = 25.0;

// Motor Consts
const int STEPS_PER_REV = 4096; 
const int STEPS_PER_RACK = STEPS_PER_REV / 8; 
long currentStepPosition = 0;

// Alert Flags (Sent to App)
bool alertRotate = false;
bool alertFan = false;

// GAB Model Parameters
struct GabParams { float M0; float C; float K; float T; };

// [FIX 1] Updated GAB Parameters for Green Coffee (Resende et al.)
// Higher M0 and lower C gives more realistic tropical EMC (approx 15-17% at 80% RH)
const GabParams gabTable[] = {
  {8.85, 6.45, 0.85, 25.0}, 
  {8.50, 6.00, 0.87, 35.0}, 
  {8.00, 5.50, 0.89, 45.0} 
};

// ------------------ HELPER FUNCTIONS ------------------

void initRackMultipliers() {
  rack_k_multiplier[0] = 1.00; rack_k_multiplier[1] = 0.95;
  rack_k_multiplier[2] = 0.90; rack_k_multiplier[3] = 0.85;
  rack_k_multiplier[4] = 0.85; rack_k_multiplier[5] = 0.90;
  rack_k_multiplier[6] = 0.95; rack_k_multiplier[7] = 1.00; 
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
  if (a_w >= 0.99f) a_w = 0.99f; 
  GabParams p = interpGabParams(T);
  float denom = (1.0f - p.K * a_w);
  if (denom == 0.0f) denom = 1e-6;
  return (p.M0 * p.C * p.K * a_w) / (denom * (denom + p.C * p.K * a_w));
}

void updateRackMoistures(float Me, float dt_min) {
  for (int i = 0; i < RACK_COUNT; i++) {
    float k_eff = k_drying * rack_k_multiplier[i]; 
    float decay = expf(-k_eff * dt_min);
    // Note: This equation allows M to increase if Me > rackM[i] (Re-wetting)
    rackM[i] = Me + (rackM[i] - Me) * decay;
  }
}

int calculatePredTime(float M_current, float Me) {
  if (M_current <= M_TARGET) return 0; 
  
  // If Me is higher than Target, we will NEVER reach Target in these conditions
  if (Me >= M_TARGET) return -1; // -1 indicates "Wetting/Stalled" condition
  
  float numerator = M_TARGET - Me;
  float denominator = M_current - Me;
  if (denominator <= 0) return -1; 

  float ratio = numerator / denominator;
  if (ratio <= 0) return 0; 

  return (int)((-1.0f / k_drying) * log(ratio));
}

// --- Stepper Logic ---
void stepMotor(int stepIndex) {
  digitalWrite(in1, seq[stepIndex][0]);
  digitalWrite(in2, seq[stepIndex][1]);
  digitalWrite(in3, seq[stepIndex][2]);
  digitalWrite(in4, seq[stepIndex][3]);
}

float getAngle() {
  long pos = currentStepPosition % STEPS_PER_REV;
  if (pos < 0) pos += STEPS_PER_REV;
  return (pos * 360.0f) / STEPS_PER_REV;
}

void rotateStepper(int steps, bool clockwise = true) {
  if (clockwise) {
    for (int i = 0; i < steps; i++) { stepMotor(i % 8); delay(stepDelay); }
    currentStepPosition += steps;
  } else {
    for (int i = steps - 1; i >= 0; i--) { stepMotor(i % 8); delay(stepDelay); }
    currentStepPosition -= steps;
  }
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

void updateLogicalPosition(int slotsMoved, bool clockwise) {
  for(int s = 0; s < slotsMoved; s++) {
    if (clockwise) {
      float lastM = rackM[RACK_COUNT - 1];
      for (int i = RACK_COUNT - 1; i > 0; i--) rackM[i] = rackM[i - 1];
      rackM[0] = lastM;
      currentRack--;
      if (currentRack < 1) currentRack = RACK_COUNT; 
    } else {
      float firstM = rackM[0];
      for (int i = 0; i < RACK_COUNT - 1; i++) rackM[i] = rackM[i + 1];
      rackM[RACK_COUNT - 1] = firstM;
      currentRack++;
      if (currentRack > RACK_COUNT) currentRack = 1;
    }
  }
}

// --- CORE LOGIC (Superset Mode Handling) ---
void evaluateSystemLogic(float T, float H) {
  unsigned long now = millis();
  
  // 1. Determine Rotation Interval (Logic runs in all modes)
  unsigned long interval;
  if (T > TEMP_HOT) interval = 30 * 1000UL;            // >30C: 30s
  else if (T >= TEMP_COOL) interval = 2 * 60 * 1000UL; // 25-30C: 2m
  else interval = 10 * 60 * 1000UL;                    // <25C: 10m

  // Check if Rotation is Due
  if (now - lastRotationTime > interval) {
    alertRotate = true; 
  }

  // [FIX 3] Fan Logic based on EMC vs Bean Moisture
  float emc = calculateEMC(T, H);
  
  // Calculate average bean moisture
  float avgM = 0;
  for(float m : rackM) avgM += m;
  avgM /= RACK_COUNT;

  // Hysteresis of 0.5%
  if (emc < (avgM - 0.5)) {
    // Air is significantly drier than beans -> Fan ON
    alertFan = true;
  } else if (emc > avgM) {
    // Air is wetter than beans -> Fan OFF (prevent re-wetting)
    alertFan = false;
  }
  // else: inside deadband, keep previous state

  // 3. AUTO MODE: Act on the alerts immediately
  if (currentMode == AUTO) {
    // Handle Rotation
    if (alertRotate) {
      Serial.println("Auto: Rotating 180 deg...");
      rotateStepper(2048, true); 
      updateLogicalPosition(4, true);
      lastRotationTime = millis(); 
      alertRotate = false; 
    }

    // Handle Fan
    if (alertFan) {
      digitalWrite(FAN_PIN, FAN_ON);
    } else {
      digitalWrite(FAN_PIN, FAN_OFF);
    }
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, FAN_OFF); // Start OFF

  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) { while (1); }
  LoRa.setTxPower(20); LoRa.setSpreadingFactor(10); LoRa.setSignalBandwidth(125E3); LoRa.enableCrc(); LoRa.receive();

  if (!bme.begin(0x76, &Wire)) { while (1); }

  initRackMultipliers();
  for (int i = 0; i < RACK_COUNT; i++) rackM[i] = initialMoisturePercent;
  
  lastRotationTime = millis();
  Serial.println("Sensor Node Ready.");
}

void sendStatusPacket() {
  float T = bme.readTemperature();
  float H = bme.readHumidity();
  float emc = calculateEMC(T, H);
  
  bool isFanOn = (digitalRead(FAN_PIN) == FAN_ON); 
  float currentAngle = getAngle();

  float avgM = 0;
  for(float m : rackM) avgM += m;
  avgM /= RACK_COUNT;

  int predMins = calculatePredTime(avgM, emc);

  String payload = "STATUS:T=" + String(T, 2);
  payload += ";H=" + String(H, 2);
  payload += ";EMC=" + String(emc, 2);
  payload += ";RACK=" + String(currentRack);
  payload += ";ANG=" + String(currentAngle, 1);
  payload += ";FAN=" + String(isFanOn ? "1" : "0");
  payload += ";PRED=" + String(predMins);
  payload += ";MODE=" + String(currentMode);
  
  int alertCode = (alertRotate ? 1 : 0) + (alertFan ? 2 : 0);
  payload += ";ALERT=" + String(alertCode); 
  
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
  }
  else if (cmd == "RESUMEAUTO") {
    autoSendEnabled = true;
  }
  else if (cmd == "FANON") {
    digitalWrite(FAN_PIN, FAN_ON); 
    sendStatusPacket(); 
  } 
  else if (cmd == "FANOFF") {
    digitalWrite(FAN_PIN, FAN_OFF); 
    sendStatusPacket();
  }
  else if (cmd == "NEXT") {
    rotateStepper(STEPS_PER_RACK, true); 
    updateLogicalPosition(1, true);
    lastRotationTime = millis(); 
    alertRotate = false; 
    sendStatusPacket();
  }
  else if (cmd.startsWith("STEP:")) {
    int angle = cmd.substring(5).toInt();
    bool cw = angle >= 0;
    int absAngle = abs(angle);
    int steps = absAngle * (STEPS_PER_REV / 360.0);
    rotateStepper(steps, cw);
    int slots = (absAngle + 22) / 45; 
    if (slots > 0) updateLogicalPosition(slots, cw);
    
    lastRotationTime = millis();
    alertRotate = false;
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

void loop() {
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

  unsigned long now = millis();
  float T = bme.readTemperature();
  float H = bme.readHumidity();
  
  // Run Core Logic 
  evaluateSystemLogic(T, H);

  if (autoSendEnabled && (now - lastSend) >= SEND_INTERVAL) {
    lastSend = now;
    float emc = calculateEMC(T, H);
    float dt_min = (float)SEND_INTERVAL / 60000.0f;
    updateRackMoistures(emc, dt_min);
    sendStatusPacket(); 
    Serial.printf("T:%.2f H:%.2f EMC:%.2f Pred:%d Alert:%d\n", T, H, emc, calculatePredTime(initialMoisturePercent, emc), (alertRotate?1:0)+(alertFan?2:0));
  }
}