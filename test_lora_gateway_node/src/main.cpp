// [File: alynxneko/greenhouse-esp/Greenhouse-ESP-ce21ed56fcbaa14510252b1a6bbbb5005a9de09e/test_lora_gateway_node/src/main.cpp]
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "BluetoothSerial.h"
#include <deque> // Include for buffer

#define LORA_CS   5
#define LORA_RST  14
#define LORA_DIO0 26

BluetoothSerial SerialBT;

// Buffer for offline data (max 50 lines to save RAM)
std::deque<String> offlineBuffer;
const int MAX_BUFFER_SIZE = 50;

void setup() {
  Serial.begin(115200);
  
  SerialBT.begin("Greenhouse_Gateway"); 
  Serial.println("Bluetooth Started! Ready to pair.");

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
  Serial.println("LoRa init OK");
}

void loop() {
  // --- 1. Check BT Connection & Sync ---
  if (SerialBT.hasClient()) {
    // If we have data buffered, dump it to the phone
    if (!offlineBuffer.empty()) {
      Serial.println("Syncing offline data...");
      while (!offlineBuffer.empty()) {
        SerialBT.println(offlineBuffer.front());
        offlineBuffer.pop_front();
        delay(20); // Small delay to not flood BT
      }
    }
  }

  // --- 2. Bluetooth -> LoRa ---
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      LoRa.beginPacket();
      LoRa.print(cmd);
      LoRa.endPacket();
      LoRa.receive(); 
    }
  }

  // --- 3. LoRa -> Gateway -> (BT or Buffer) ---
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String msg = "";
    while (LoRa.available()) {
      msg += (char)LoRa.read();
    }
    msg.trim();

    if (msg.length() > 0) {
      int rssi = LoRa.packetRssi();
      float snr = LoRa.packetSnr();
      msg += ";RSSI=" + String(rssi);
      msg += ";SNR=" + String(snr, 2);
      
      Serial.println("Relay: " + msg);

      if (SerialBT.hasClient()) {
        SerialBT.println(msg); 
      } else {
        // Save to buffer if offline
        if (offlineBuffer.size() >= MAX_BUFFER_SIZE) {
          offlineBuffer.pop_front(); // Remove oldest
        }
        offlineBuffer.push_back(msg);
        Serial.println("Buffered offline message.");
      }
    }
  }
}