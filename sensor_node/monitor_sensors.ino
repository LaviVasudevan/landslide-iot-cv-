/*
==============================================
ARDUINO UNO - Landslide Transmitter with Real Sensors
Sends: "A0", "A1", or "A2" (A = Arduino source)
==============================================
*/

#include <SPI.h>
#include <LoRa.h>

// LoRa Pins
#define SS_PIN 10
#define RST_PIN 9
#define DIO0_PIN 2

// Sensor Pins (Your Actual Connections)
#define RAIN_GAUGE_PIN 7    // D7 - Digital input
#define MOISTURE_PIN A0      // A0 - Analog input
#define VIBRATION_PIN 6      // D6 - Digital input
#define TILT_PIN 5           // D5 - Digital input

// Thresholds
#define RAINFALL_THRESHOLD 1.0      // Percentage or tips
#define MOISTURE_THRESHOLD 60.0      // Percentage (0-100%)
#define VIBRATION_THRESHOLD 1        // Digital: 1 = vibration detected
#define TILT_THRESHOLD 1             // Digital: 1 = tilt detected

unsigned int count = 0;
bool firstTx = true;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  // Setup sensor pins
  pinMode(RAIN_GAUGE_PIN, INPUT);
  pinMode(VIBRATION_PIN, INPUT);
  pinMode(TILT_PIN, INPUT);
  // A0 is analog, no pinMode needed
  
  // Setup LoRa
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);
  
  randomSeed(analogRead(A1));  // Use unused analog pin for random seed
  
  Serial.println("====================================");
  Serial.println("  Arduino Landslide Transmitter");
  Serial.println("  Source ID: A");
  Serial.println("====================================");
  Serial.println("LoRa Ready (433MHz, SF7)");
  Serial.println("\nSensor Pins:");
  Serial.println("  Rain Gauge: D7");
  Serial.println("  Moisture:   A0");
  Serial.println("  Vibration:  D6");
  Serial.println("  Tilt:       D5");
  Serial.println("\nStarting monitoring...\n");
}

void loop() {
  // Read all sensors
  int rainDetected = digitalRead(RAIN_GAUGE_PIN);        // 0 or 1
  int moistureRaw = analogRead(MOISTURE_PIN);            // 0-1023
  int vibrationDetected = digitalRead(VIBRATION_PIN);    // 0 or 1
  int tiltDetected = digitalRead(TILT_PIN);              // 0 or 1
  
  // Convert moisture to percentage (0-100%)
  // Adjust these values based on your sensor calibration
  // Typical: 0 (in water) to 1023 (in air) - needs inversion
  float moisturePercent = map(moistureRaw, 1023, 0, 0, 100);
  moisturePercent = constrain(moisturePercent, 0, 100);
  
  // Simulate rainfall value (you can modify this based on rain gauge type)
  // For tipping bucket: count tips over time window
  float rainfallValue = rainDetected ? 100 : random(0, 50);
  
  // Count alerts based on thresholds
  int alertCount = 0;
  
  if (rainfallValue > RAINFALL_THRESHOLD) alertCount++;
  if (moisturePercent > MOISTURE_THRESHOLD) alertCount++;
  if (vibrationDetected == VIBRATION_THRESHOLD) alertCount++;
  if (tiltDetected == TILT_THRESHOLD) alertCount++;

  // Create packet: "A" + zone number
  String packet = "S" + String(alertCount);
  
  // Send via LoRa
  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();
  
  count++;
  
  // Print detailed status to Serial Monitor
  Serial.println("========================================");
  Serial.print("Transmission #");
  Serial.println(count);
  Serial.println("----------------------------------------");
  Serial.println("Sensor Readings:");
  Serial.print("  Rain:      ");
  Serial.print(rainfallValue, 1);
  Serial.print("%");
  if (rainfallValue > RAINFALL_THRESHOLD) Serial.print(" [ALERT]");
  Serial.println();
  
  Serial.print("  Moisture:  ");
  Serial.print(moisturePercent, 1);
  Serial.print("% (Raw: ");
  Serial.print(moistureRaw);
  Serial.print(")");
  if (moisturePercent > MOISTURE_THRESHOLD) Serial.print(" [ALERT]");
  Serial.println();
  
  Serial.print("  Vibration: ");
  Serial.print(vibrationDetected ? "DETECTED" : "Normal");
  if (vibrationDetected == VIBRATION_THRESHOLD) Serial.print(" [ALERT]");
  Serial.println();
  
  Serial.print("  Tilt:      ");
  Serial.print(tiltDetected ? "DETECTED" : "Normal");
  if (tiltDetected == TILT_THRESHOLD) Serial.print(" [ALERT]");
  Serial.println();
  
  Serial.println("----------------------------------------");
  Serial.print("Alert Count: ");
  Serial.print(alertCount);
  Serial.println("/4");
    
  Serial.print("Transmitted: ");
  Serial.println(packet);
  Serial.println("========================================\n");
  
  // Alternating delay for timing coordination with RPi5
  if (firstTx) {
    delay(5000);   // 5 seconds
    firstTx = false;
  } else {
    delay(10000);  // 10 seconds
    firstTx = true;
  }
}