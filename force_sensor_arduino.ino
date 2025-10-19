/*
 * ESP32 Xiao S3 - SingleTact 1N Force Sensor via BLE
 * Using CORRECT SingleTact protocol (6-byte read)
 * 
 * Wiring (Xiao ESP32 S3):
 *   SingleTact VCC → 3.3V
 *   SingleTact GND → GND
 *   SingleTact SDA → GPIO 5 (D4)
 *   SingleTact SCL → GPIO 6 (D5)
 */

#include <Wire.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// SingleTact Configuration
const uint8_t SENSOR_ADDRESS = 0x04;
const int SDA_PIN = 5;
const int SCL_PIN = 6;

// SingleTact 1N Sensor Specs (based on working protocol)
const float SENSOR_MAX_FORCE = 1.0;   // 1 Newton max
const int SENSOR_MAX_VALUE = 1024;     // Raw value range 0-1024

// BLE UUIDs - Must match Python code
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
bool sensorDetected = false;

class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("\n╔═══════════════════════════════════╗");
    Serial.println("║  ✅ BLE CLIENT CONNECTED!        ║");
    Serial.println("╚═══════════════════════════════════╝");
    
    // Log connection parameters
    Serial.print("Free heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
    
    // Don't start advertising when connected
    pServer->getAdvertising()->stop();
  }
  
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("\n╔═══════════════════════════════════╗");
    Serial.println("║  ❌ BLE CLIENT DISCONNECTED      ║");
    Serial.println("╚═══════════════════════════════════╝");
    
    // Log disconnect reason
    Serial.print("Reason: Check signal strength & power\n");
    Serial.print("Free heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
    
    Serial.println("Restarting advertising in 500ms...\n");
    
    // Small delay before restarting advertising
    delay(500);
    pServer->getAdvertising()->start();
  }
};

void scanI2C() {
  Serial.println("🔍 Scanning I2C bus...");
  
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("   ✅ Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      if (addr == 0x04) {
        Serial.print(" ← SingleTact Sensor!");
        sensorDetected = true;
      }
      Serial.println();
    }
  }
  
  if (!sensorDetected) {
    Serial.println("   ❌ No devices found");
  }
  Serial.println();
}

// CORRECT SingleTact Reading Protocol - NON-BLOCKING VERSION
int readSingleTact() {
  // Step 1: Send command to sensor
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(0x00);  // Command byte
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    return -1;  // Communication error
  }
  
  // Step 2: Wait for sensor to process (NON-BLOCKING)
  // Instead of delay(10), use millis() timing
  unsigned long startTime = millis();
  while (millis() - startTime < 5) {  // 5ms wait instead of 10ms
    yield();  // Allow BLE and other tasks to run
    delayMicroseconds(100);  // Small delay
  }
  
  // Step 3: Read 6 bytes from sensor with timeout
  unsigned long readStart = millis();
  while (Wire.available() < 6 && millis() - readStart < 10) {  // 10ms read timeout
    Wire.requestFrom(SENSOR_ADDRESS, 6);
    yield();  // Allow BLE to run
    delayMicroseconds(100);
  }
  
  if (Wire.available() >= 6) {
    byte data[6];
    for (int i = 0; i < 6; i++) {
      data[i] = Wire.read();
    }
    
    // Step 4: Extract raw value from bytes 4 and 5
    int raw_value = (data[4] << 8) | data[5];
    
    return raw_value;
  }
  
  return -1;  // Read failed/timeout
}

// Convert raw sensor value to Newtons
float toNewtons(int raw_value) {
  if (raw_value < 0) return 0.0;
  
  // Based on working code: (raw / 1024) * 1.0N
  float force = (raw_value / (float)SENSOR_MAX_VALUE) * SENSOR_MAX_FORCE;
  
  return force;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n");
  Serial.println("╔═══════════════════════════════════════╗");
  Serial.println("║  SingleTact 1N Force Sensor - BLE    ║");
  Serial.println("║  Pressure-Sensitive Drawing Stylus   ║");
  Serial.println("╚═══════════════════════════════════════╝\n");
  
  // ===== STEP 1: INITIALIZE BLE FIRST =====
  Serial.println("─── STEP 1: BLE Initialization ───");
  Serial.println("   Device name: ForceStylus");
  
  BLEDevice::init("ForceStylus");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  
  Serial.println("   ✅ BLE ready and advertising\n");
  
  delay(500);
  
  // ===== STEP 3: INITIALIZE I2C AFTER BLE =====
  Serial.println("─── STEP 3: I2C Initialization ───");
  Serial.print("   SDA: GPIO ");
  Serial.println(SDA_PIN);
  Serial.print("   SCL: GPIO ");
  Serial.println(SCL_PIN);
  Serial.println("   Clock: 100 kHz");
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeout(5);  // 5ms I2C timeout - fast response!
  delay(100);
  
  Serial.println("   ✅ I2C initialized with fast timeout protection\n");
  
  // ===== STEP 4: SCAN FOR SENSOR =====
  Serial.println("─── STEP 4: Sensor Detection ───");
  scanI2C();
  
  // ===== STEP 5: TEST READINGS =====
  if (sensorDetected) {
    Serial.println("─── STEP 5: Sensor Test ───");
    Serial.println("   Using CORRECT 6-byte protocol");
    Serial.println("   Format: Raw Value (Newtons)\n");
    
    for (int i = 0; i < 5; i++) {
      int raw = readSingleTact();
      Serial.print("   Test ");
      Serial.print(i+1);
      Serial.print(": ");
      
      if (raw >= 0) {
        float force = toNewtons(raw);
        Serial.print(raw);
        Serial.print(" (");
        Serial.print(force, 3);
        Serial.println(" N) ✓");
      } else {
        Serial.println("ERROR ✗");
      }
    }
    
    Serial.println("\n   💡 Expected ranges:");
    Serial.println("      No pressure:  0-50    (0.00-0.05 N)");
    Serial.println("      Light touch:  100-300 (0.10-0.30 N)");
    Serial.println("      Full press:   900-1024 (0.90-1.00 N)");
  }
  
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║  🚀 SYSTEM READY                     ║");
  Serial.println("╚═══════════════════════════════════════╝\n");
  
  if (sensorDetected) {
    Serial.println("✅ Sensor working with correct protocol!");
    Serial.println("📱 Waiting for BLE connection...");
    Serial.println("💡 Press sensor to see live values\n");
  } else {
    Serial.println("⚠️  No sensor detected");
    Serial.println("   Will send test data for debugging\n");
  }
}

void loop() {
  static int lastForce = -999;
  
  if (sensorDetected) {
    int raw = readSingleTact();
    if (raw < 0) raw = 0;
    
    // Print if value changed significantly
    if (abs(raw - lastForce) > 10) {
      float force = toNewtons(raw);
      
      // Format output nicely
      Serial.print("Raw: ");
      if (raw < 10) Serial.print("   ");
      else if (raw < 100) Serial.print("  ");
      else if (raw < 1000) Serial.print(" ");
      Serial.print(raw);
      
      Serial.print(" | ");
      Serial.print(force, 3);
      Serial.print(" N");
      
      // Visual bar (percentage of max)
      int bars = (raw * 40) / SENSOR_MAX_VALUE;
      Serial.print(" │");
      for (int i = 0; i < bars; i++) {
        Serial.print("█");
      }
      
      if (deviceConnected) {
        Serial.print(" ← BLE");
      }
      
      Serial.println();
      lastForce = raw;
    }
    
    // Send via BLE if connected
    if (deviceConnected) {
      uint8_t data[2] = {raw & 0xFF, (raw >> 8) & 0xFF};
      pCharacteristic->setValue(data, 2);
      pCharacteristic->notify();
    }
    
    delay(100);  // Match 10Hz update rate
    
  } else {
    // No sensor - send test pattern
    if (deviceConnected) {
      int testVal = (millis() / 100) % 1024;
      uint8_t data[2] = {testVal & 0xFF, (testVal >> 8) & 0xFF};
      pCharacteristic->setValue(data, 2);
      pCharacteristic->notify();
      
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 3000) {
        Serial.print("⚠️  Test mode: ");
        Serial.println(testVal);
        lastPrint = millis();
      }
    }
    delay(200);  // Match sensor mode rate
  }
}