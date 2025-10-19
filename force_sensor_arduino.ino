/*
 * ESP32 Supermini - SingleTact Force Sensor via BLE
 * Reads SingleTact 1N sensor over I2C and broadcasts via Bluetooth Low Energy
 * 
 * Wiring:
 *   SingleTact VCC → ESP32 3.3V
 *   SingleTact GND → ESP32 GND
 *   SingleTact SDA → ESP32 GPIO 21
 *   SingleTact SCL → ESP32 GPIO 22
 */

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>

// ===== CONFIGURATION =====
// SingleTact I2C Settings
const uint8_t SINGLETACT_ADDR = 0x04;  // Default I2C address
const int SDA_PIN = 21;                // I2C Data pin
const int SCL_PIN = 22;                // I2C Clock pin

// BLE UUIDs - Must match Python code
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Update rate
const int UPDATE_RATE_MS = 50;  // Send data every 50ms (20Hz)

// ===== GLOBAL VARIABLES =====
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool sensorDetected = false;

// Statistics
unsigned long lastReadTime = 0;
unsigned long readCount = 0;
int lastForceValue = 0;

// ===== BLE CALLBACKS =====
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("\n╔════════════════════════════════════╗");
      Serial.println("║  ✅ BLE CLIENT CONNECTED!         ║");
      Serial.println("╚════════════════════════════════════╝");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("\n╔════════════════════════════════════╗");
      Serial.println("║  ❌ BLE CLIENT DISCONNECTED       ║");
      Serial.println("╚════════════════════════════════════╝");
      Serial.println("Restarting BLE advertising...");
      
      BLEDevice::startAdvertising();
      Serial.println("✅ Ready for new connection\n");
    }
};

// ===== I2C SCANNER =====
void scanI2C() {
  Serial.println("\n🔍 Scanning I2C bus...");
  Serial.println("   Address range: 0x01 to 0x7F");
  
  int devicesFound = 0;
  
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("   ✅ Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      if (address == SINGLETACT_ADDR) {
        Serial.println(" ← SingleTact Sensor!");
        sensorDetected = true;
      } else {
        Serial.println();
      }
      devicesFound++;
    }
  }
  
  Serial.print("\n   Total devices found: ");
  Serial.println(devicesFound);
  
  if (devicesFound == 0) {
    Serial.println("   ⚠️  No I2C devices detected!");
    Serial.println("   → Check wiring (SDA, SCL, VCC, GND)");
  }
  Serial.println();
}

// ===== READ SINGLETACT SENSOR =====
int readSingleTact() {
  Wire.beginTransmission(SINGLETACT_ADDR);
  Wire.write(0x00);  // Request data from register 0
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    // Communication error
    return -1;
  }
  
  // Request 2 bytes (16-bit force value)
  int bytesReceived = Wire.requestFrom(SINGLETACT_ADDR, 2);
  
  if (bytesReceived >= 2) {
    uint8_t lowByte = Wire.read();
    uint8_t highByte = Wire.read();
    
    // Combine into 16-bit value
    int forceValue = (highByte << 8) | lowByte;
    readCount++;
    return forceValue;
  }
  
  return -1;  // Read failed
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial to stabilize
  
  // Print header
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║  ESP32 SingleTact Force Sensor via BLE    ║");
  Serial.println("║  Pressure-Sensitive Drawing System        ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  
  // ===== INITIALIZE I2C =====
  Serial.println("─── STEP 1: I2C Initialization ───");
  Serial.print("   SDA Pin: GPIO ");
  Serial.println(SDA_PIN);
  Serial.print("   SCL Pin: GPIO ");
  Serial.println(SCL_PIN);
  Serial.println("   Clock: 100 kHz");
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // 100kHz I2C clock
  delay(100);
  
  Serial.println("   ✅ I2C bus initialized\n");
  
  // ===== SCAN FOR DEVICES =====
  Serial.println("─── STEP 2: I2C Device Scan ───");
  scanI2C();
  
  // ===== TEST SINGLETACT =====
  Serial.println("─── STEP 3: SingleTact Sensor Test ───");
  
  if (sensorDetected) {
    Serial.println("   Testing sensor readings...");
    
    // Take 5 test readings
    int validReadings = 0;
    for (int i = 0; i < 5; i++) {
      int testValue = readSingleTact();
      Serial.print("   Test ");
      Serial.print(i + 1);
      Serial.print(": ");
      
      if (testValue >= 0) {
        Serial.print(testValue);
        Serial.println(" ✓");
        validReadings++;
      } else {
        Serial.println("ERROR ✗");
      }
      delay(100);
    }
    
    Serial.print("\n   Valid readings: ");
    Serial.print(validReadings);
    Serial.println("/5");
    
    if (validReadings >= 3) {
      Serial.println("   ✅ Sensor working correctly!\n");
    } else {
      Serial.println("   ⚠️  Sensor communication issues detected\n");
    }
  } else {
    Serial.println("   ❌ SingleTact not found at 0x04");
    Serial.println("   → Check wiring");
    Serial.println("   → Verify sensor I2C address");
    Serial.println("   → Try power cycling the sensor\n");
  }
  
  // ===== INITIALIZE BLE =====
  Serial.println("─── STEP 4: BLE Initialization ───");
  Serial.println("   Device Name: ForceStylus");
  
  BLEDevice::init("ForceStylus");
  Serial.println("   ✅ BLE device created");
  
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  Serial.println("   ✅ BLE server created");
  
  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  Serial.println("   ✅ Service created");
  
  // Create BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  Serial.println("   ✅ Characteristic created");
  
  // Start service
  pService->start();
  Serial.println("   ✅ Service started");
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
  
  Serial.println("   ✅ Advertising started\n");
  
  // ===== READY =====
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║           🚀 SYSTEM READY!                ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println("\n📱 Waiting for BLE connection from computer...");
  Serial.println("   Device is discoverable as: ForceStylus\n");
  
  if (sensorDetected) {
    Serial.println("💡 Try pressing the sensor - values will appear below:");
  } else {
    Serial.println("⚠️  No sensor detected - will send test values");
  }
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
}

// ===== MAIN LOOP =====
void loop() {
  unsigned long currentTime = millis();
  
  // Only send data at specified rate
  if (currentTime - lastReadTime >= UPDATE_RATE_MS) {
    lastReadTime = currentTime;
    
    int forceValue;
    
    // Read sensor (or use test value if not connected)
    if (sensorDetected) {
      forceValue = readSingleTact();
      
      if (forceValue < 0) {
        // Communication error - use last known value
        forceValue = lastForceValue;
      }
    } else {
      // No sensor - send test pattern for debugging
      forceValue = 0;  // Or use: (millis() / 100) % 1000 for animated test
    }
    
    // Only print if value changed significantly (reduce spam)
    if (abs(forceValue - lastForceValue) > 10 || (forceValue > 0 && lastForceValue == 0) || (forceValue == 0 && lastForceValue > 0)) {
      
      // Print reading with visual bar
      Serial.print("Force: ");
      if (forceValue < 10) Serial.print("   ");
      else if (forceValue < 100) Serial.print("  ");
      else if (forceValue < 1000) Serial.print(" ");
      Serial.print(forceValue);
      Serial.print(" │");
      
      // Visual bar (scale to ~40 chars max)
      int barLength = map(forceValue, 0, 1000, 0, 40);
      for (int i = 0; i < barLength; i++) {
        Serial.print("█");
      }
      
      // Connection status
      if (deviceConnected) {
        Serial.print(" ← BLE CONNECTED");
      }
      
      Serial.println();
      
      lastForceValue = forceValue;
    }
    
    // Send via BLE if connected
    if (deviceConnected) {
      // Convert to 2-byte array
      uint8_t forceData[2];
      forceData[0] = forceValue & 0xFF;         // Low byte
      forceData[1] = (forceValue >> 8) & 0xFF;  // High byte
      
      // Send notification
      pCharacteristic->setValue(forceData, 2);
      pCharacteristic->notify();
    }
  }
  
  // Print stats every 10 seconds
  static unsigned long lastStatsTime = 0;
  if (currentTime - lastStatsTime >= 10000 && deviceConnected && readCount > 0) {
    lastStatsTime = currentTime;
    
    Serial.println("\n📊 Stats:");
    Serial.print("   Readings: ");
    Serial.println(readCount);
    Serial.print("   Rate: ~");
    Serial.print(1000 / UPDATE_RATE_MS);
    Serial.println(" Hz");
    Serial.print("   Connection: ");
    Serial.println(deviceConnected ? "Active" : "None");
    Serial.println();
  }
}