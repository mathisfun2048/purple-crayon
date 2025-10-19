/*
 * ESP32 Xiao S3 - SingleTact 1N Force Sensor via BLE
 * FIXED VERSION - Stable BLE with proper task separation
 * 
 * Wiring (Xiao ESP32 S3):
 *   SingleTact VCC → 3.3V
 *   SingleTact GND → GND
 *   SingleTact SDA → GPIO 5 (D4)
 *   SingleTact SCL → GPIO 6 (D5)
 */

#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// SingleTact Configuration
const uint8_t SENSOR_ADDRESS = 0x04;
const int SDA_PIN = 5;
const int SCL_PIN = 6;

// SingleTact 1N Sensor Specs
const float SENSOR_MAX_FORCE = 1.0;   // 1 Newton max
const int SENSOR_MAX_VALUE = 1024;    // Raw value range 0-1024

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Global variables
BLECharacteristic *pCharacteristic;
BLEServer *pServer;
volatile bool deviceConnected = false;
volatile bool oldDeviceConnected = false;
bool sensorDetected = false;

// Sensor data (thread-safe access)
SemaphoreHandle_t dataMutex;
volatile int currentRawValue = 0;
volatile bool newDataAvailable = false;

// Task handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t bleTaskHandle = NULL;

class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("\n✅ BLE CLIENT CONNECTED!");
    
    // Request connection parameter update for stability
    // Min interval: 7.5ms, Max interval: 20ms, Latency: 0, Timeout: 400ms
    pServer->updateConnParams(0, 6, 16, 0, 400);
  }
  
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("\n❌ BLE CLIENT DISCONNECTED");
  }
};

// I2C communication with sensor (runs in separate task)
int readSingleTactNonBlocking() {
  // Send command
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    return -1;
  }
  
  // Use vTaskDelay instead of delay for proper FreeRTOS timing
  vTaskDelay(pdMS_TO_TICKS(5));
  
  // Request data with timeout
  Wire.requestFrom(SENSOR_ADDRESS, 6);
  
  unsigned long timeout = millis() + 10;
  while (Wire.available() < 6 && millis() < timeout) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  
  if (Wire.available() >= 6) {
    byte data[6];
    for (int i = 0; i < 6; i++) {
      data[i] = Wire.read();
    }
    return (data[4] << 8) | data[5];
  }
  
  return -1;
}

// Task 1: Handle sensor reading (Core 0)
void sensorTask(void *pvParameters) {
  Serial.println("Sensor task started on core 0");
  
  while (1) {
    if (sensorDetected) {
      int raw = readSingleTactNonBlocking();
      
      if (raw >= 0) {
        // Update shared data with mutex protection
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          currentRawValue = raw;
          newDataAvailable = true;
          xSemaphoreGive(dataMutex);
        }
      }
      
      // Sensor read rate: 20Hz (50ms)
      vTaskDelay(pdMS_TO_TICKS(50));
    } else {
      // Test mode - generate dummy data
      static int testVal = 0;
      testVal = (testVal + 10) % 1024;
      
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        currentRawValue = testVal;
        newDataAvailable = true;
        xSemaphoreGive(dataMutex);
      }
      
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Feed watchdog
    esp_task_wdt_reset();
  }
}

// Task 2: Handle BLE communication (Core 1)
void bleTask(void *pvParameters) {
  Serial.println("BLE task started on core 1");
  
  while (1) {
    // Handle connection state changes
    if (deviceConnected != oldDeviceConnected) {
      if (!deviceConnected && oldDeviceConnected) {
        vTaskDelay(pdMS_TO_TICKS(500));
        pServer->startAdvertising();
        Serial.println("Restarted advertising");
      }
      oldDeviceConnected = deviceConnected;
    }
    
    // Send data if connected and new data available
    if (deviceConnected && newDataAvailable) {
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        int value = currentRawValue;
        newDataAvailable = false;
        xSemaphoreGive(dataMutex);
        
        // Send via BLE
        uint8_t data[2] = {value & 0xFF, (value >> 8) & 0xFF};
        pCharacteristic->setValue(data, 2);
        pCharacteristic->notify();
        
        // Optional: Print for debugging
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 1000) {
          float force = (value / (float)SENSOR_MAX_VALUE) * SENSOR_MAX_FORCE;
          Serial.print("Force: ");
          Serial.print(force, 3);
          Serial.println(" N");
          lastPrint = millis();
        }
      }
    }
    
    // BLE task rate: 10ms (allows quick response to notifications)
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Feed watchdog
    esp_task_wdt_reset();
  }
}

void scanI2C() {
  Serial.println("Scanning I2C bus...");
  
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      if (addr == 0x04) {
        Serial.print(" ← SingleTact Sensor!");
        sensorDetected = true;
      }
      Serial.println();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n═══════════════════════════════════════");
  Serial.println("  SingleTact Force Sensor - BLE (FIXED)");
  Serial.println("═══════════════════════════════════════\n");
  
  // Create mutex for thread-safe data access
  dataMutex = xSemaphoreCreateMutex();
  
  // Initialize I2C
  Serial.println("Initializing I2C...");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeout(10);  // Increased timeout for stability
  delay(100);
  
  // Scan for sensor
  scanI2C();
  
  // Initialize BLE
  Serial.println("\nInitializing BLE...");
  BLEDevice::init("ForceStylus");
  
  // Configure BLE for lower power and better stability
  BLEDevice::setPower(ESP_PWR_LVL_P9);  // Max power for stability
  BLEDevice::setMTU(517);  // Larger MTU for efficiency
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | 
    BLECharacteristic::PROPERTY_NOTIFY
  );
  
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  
  // Configure advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // 7.5ms
  pAdvertising->setMaxPreferred(0x12);  // 15ms
  pAdvertising->start();
  
  Serial.println("BLE ready and advertising");
  
  // Create tasks on separate cores
  xTaskCreatePinnedToCore(
    sensorTask,           // Function
    "SensorTask",         // Name
    4096,                 // Stack size
    NULL,                 // Parameters
    1,                    // Priority
    &sensorTaskHandle,    // Handle
    0                     // Core 0
  );
  
  xTaskCreatePinnedToCore(
    bleTask,              // Function
    "BLETask",            // Name
    4096,                 // Stack size
    NULL,                 // Parameters
    2,                    // Priority (higher for BLE)
    &bleTaskHandle,       // Handle
    1                     // Core 1
  );
  
  Serial.println("\n🚀 SYSTEM READY");
  Serial.println("📱 Waiting for BLE connection...\n");
}

void loop() {
  // Main loop kept minimal - all work done in tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  // Monitor system health
  static unsigned long lastHealthCheck = 0;
  if (millis() - lastHealthCheck > 10000) {
    Serial.print("System health: Heap=");
    Serial.print(ESP.getFreeHeap());
    Serial.print(" bytes, Connected=");
    Serial.println(deviceConnected ? "Yes" : "No");
    lastHealthCheck = millis();
  }
}
