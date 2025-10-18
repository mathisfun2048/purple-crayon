"""
SingleTact 1N Force Sensor - I2C Mode (FIXED)
CircuitPython for XIAO RP2040
"""

import time
import board
import busio

# Initialize I2C on D4 (SDA) and D5 (SCL)
i2c = busio.I2C(board.SCL, board.SDA)

# SingleTact I2C address
SENSOR_ADDRESS = 0x04

print("SingleTact 1N Force Sensor - I2C Mode (Fixed)")
print("==============================================")
print()

while True:
    while not i2c.try_lock():
        pass
    
    try:
        # Prepare command buffer
        # Byte 0: 0x01 = I2C read command
        # Byte 1: 128 = Slave data offset
        # Byte 2: 6 = Number of bytes to read
        command = bytearray([0x01, 128, 6])
        
        # Send the command to the sensor
        i2c.writeto(SENSOR_ADDRESS, command)
        
        # Small delay to let sensor process
        time.sleep(0.001)
        
        # Read 6 bytes from sensor
        result = bytearray(6)
        i2c.readfrom_into(SENSOR_ADDRESS, result)
        
        # Extract the raw sensor value from bytes 4 and 5
        # High byte is at index 4, low byte at index 5
        raw_value = (result[4] << 8) | result[5]
        
        # Convert to force
        # Raw value range: 0-511 (or possibly 0-1023 for 10-bit)
        # This maps to 0-1N for the 1N sensor
        force = (raw_value / 512.0) * 1.0  # For 1N sensor
        
        # Display all data for debugging
        print(f"Raw bytes: {[hex(b) for b in result]}")
        print(f"Raw value: {raw_value:4d} | Force: {force:.3f}N")
        print()
        
    except OSError as e:
        print(f"Error: {e}")
    
    finally:
        i2c.unlock()
    
    time.sleep(0.1)  # 10Hz update rate
