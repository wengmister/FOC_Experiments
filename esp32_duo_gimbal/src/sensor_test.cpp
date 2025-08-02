#include <Arduino.h>
#include <Wire.h>

// AS5600 I2C address and registers
#define AS5600_ADDR 0x36
#define AS5600_ANGLE_H 0x0E
#define AS5600_ANGLE_L 0x0F

// Function to read AS5600 angle from specified I2C bus
uint16_t readAS5600(TwoWire &wire) {
  wire.beginTransmission(AS5600_ADDR);
  wire.write(AS5600_ANGLE_H);
  if (wire.endTransmission() != 0) {
    return 0xFFFF; // Error indicator
  }
  
  wire.requestFrom(AS5600_ADDR, 2);
  if (wire.available() >= 2) {
    uint16_t angle = wire.read() << 8;
    angle |= wire.read();
    return angle & 0x0FFF; // 12-bit value
  }
  return 0xFFFF; // Error indicator
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("Direct AS5600 I2C Test");
  Serial.println("ESP32-S3 Feather with dual I2C buses");
  
  // Initialize I2C buses
  Wire.begin(3, 4);      // I2C bus 0 - SDA=3, SCL=4
  Wire1.begin(A0, A1);   // I2C bus 1 - SDA=A0, SCL=A1
  
  Serial.println("I2C buses initialized");
  Serial.println("Testing sensor connections...");
  
  // Test initial readings
  uint16_t test1 = readAS5600(Wire);
  uint16_t test2 = readAS5600(Wire1);
  
  Serial.print("Sensor 1 (Wire): ");
  Serial.println(test1 == 0xFFFF ? "ERROR" : "OK");
  Serial.print("Sensor 2 (Wire1): ");
  Serial.println(test2 == 0xFFFF ? "ERROR" : "OK");
  
  Serial.println("Starting continuous readings...");
  Serial.println("Format: Raw1, Deg1, Raw2, Deg2");
}

void loop() {
  // Read both sensors
  uint16_t raw1 = readAS5600(Wire);
  uint16_t raw2 = readAS5600(Wire1);
  
  // Convert to degrees (12-bit = 4096 counts = 360°)
  float deg1 = (raw1 == 0xFFFF) ? -1 : (raw1 * 360.0 / 4096.0);
  float deg2 = (raw2 == 0xFFFF) ? -1 : (raw2 * 360.0 / 4096.0);
  
  // Print readings
  Serial.print("Raw1: ");
  Serial.print(raw1);
  Serial.print(" (");
  if (deg1 == -1) {
    Serial.print("ERROR");
  } else {
    Serial.print(deg1, 2);
    Serial.print("°");
  }
  Serial.print(") | Raw2: ");
  Serial.print(raw2);
  Serial.print(" (");
  if (deg2 == -1) {
    Serial.print("ERROR");
  } else {
    Serial.print(deg2, 2);
    Serial.print("°");
  }
  Serial.println(")");
  
  delay(100); // 10Hz update rate
}