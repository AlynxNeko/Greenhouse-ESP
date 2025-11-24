#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "BluetoothSerial.h"

#define LORA_CS   5
#define LORA_RST  14
#define LORA_DIO0 26

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  
  // 1. Initialize Bluetooth
  SerialBT.begin("Greenhouse_Gateway"); 
  Serial.println("Bluetooth Started! Ready to pair.");

  // 2. Initialize LoRa
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
  LoRa.receive(); // Start listening
  Serial.println("LoRa init OK");
}

void loop() {
  // --- FLOW 1: Flutter (BT) -> Gateway -> Sensor (LoRa) ---
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      // Forward command to Sensor via LoRa
      LoRa.beginPacket();
      LoRa.print(cmd);
      LoRa.endPacket();
      LoRa.receive(); 
    }
  }

  // --- FLOW 2: Sensor (LoRa) -> Gateway -> Flutter (BT) ---
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String msg = "";
    while (LoRa.available()) {
      msg += (char)LoRa.read();
    }
    msg.trim();

    if (msg.length() > 0) {
      // --- TAMBAHAN: Ambil data Sinyal ---
      int rssi = LoRa.packetRssi();
      float snr = LoRa.packetSnr();

      // Append ke pesan: "...;RSSI=-80;SNR=9.25"
      msg += ";RSSI=" + String(rssi);
      msg += ";SNR=" + String(snr, 2);
      
      SerialBT.println(msg); // Kirim ke Flutter
      Serial.println("Relay: " + msg);
    }
  }
}