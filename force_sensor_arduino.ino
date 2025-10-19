/*
  SingleTact 1N Force Sensor - CORRECT Protocol
  XIAO ESP32-S3
  
  Protocol:
  1. Send command to sensor
  2. Wait for processing
  3. Read 6 bytes
  4. Raw value is in bytes 4 and 5
*/

#include <Wire.h>

#define SENSOR_ADDRESS 0x04

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\nSingleTact 1N Force Sensor - I2C Mode");
  Serial.println("======================================");
  Serial.println();
  
  Wire.begin();
  Wire.setClock(100000);
  
  delay(100);
  Serial.println("Setup complete. Reading sensor...");
  Serial.println();
}

void loop() {
  // Step 1: Send command to sensor
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(0x00);  // Command byte (0x00 is typical read command)
  Wire.endTransmission();
  
  // Step 2: Wait for sensor to process
  delay(10);  // 10ms delay - adjust if needed
  
  // Step 3: Read 6 bytes from sensor
  Wire.requestFrom(SENSOR_ADDRESS, 6);
  
  if (Wire.available() >= 6) {
    byte data[6];
    for (int i = 0; i < 6; i++) {
      data[i] = Wire.read();
    }
    
    // Step 4: Extract raw value from bytes 4 and 5
    // High byte at index 4, low byte at index 5
    int raw_value = (data[4] << 8) | data[5];
    
    // Convert to force (0-512 maps to 0-1N)
    float force = (raw_value / (512.0*2)) * 1.0;
    
    // Display readings
    Serial.print("Raw: ");
    Serial.print(raw_value);
    Serial.print(" | Force: ");
    Serial.print(force, 3);
    Serial.println("N");
    
  } else {
    Serial.println("Error: Could not read 6 bytes from sensor");
  }
  
  delay(100);  // 10Hz update rate
}