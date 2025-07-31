#include <Arduino.h>
#include <SimpleFOC.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 33
#define NEOPIXEL_POWER 21
#define NUM_LEDS 1
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Motor instance (7 pole pairs)
BLDCMotor motor = BLDCMotor(7);

// SimpleFOC Mini driver - EN=A5, IN1/2/3=9/6/5
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 6, 5, A5);

// I2C magnetic sensor using your working code
MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0x0E, 4);

// LED status function with WORKING NeoPixel!
void updateLED() {
  if (motor.controller == MotionControlType::velocity) {
    // Velocity mode = Red
    strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red
    Serial.println("LED: Velocity mode (RED)");
  } else {
    // Position mode = Blue  
    strip.setPixelColor(0, strip.Color(0, 0, 255)); // Blue
    Serial.println("LED: Position mode (BLUE)");
  }
  strip.show();
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // Initialize WORKING NeoPixel!
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH); // Enable NeoPixel power
  
  strip.begin();
  strip.setBrightness(100); // Nice brightness
  strip.show(); // Initialize to off
  
  Serial.println("ESP32 SimpleFOC with SimpleFOC Mini");
  Serial.println("Pins: EN=A5, IN1/2/3=9/6/5");
  Serial.print("NeoPixel pin: ");
  Serial.print(LED_PIN);
  Serial.print(", Power pin: ");
  Serial.println(NEOPIXEL_POWER);
  Serial.println("LED: Red=Velocity, Blue=Position");
  
  // Test NeoPixel with victory colors!
  Serial.println("NeoPixel SUCCESS test!");
  strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red
  strip.show();
  delay(500);
  strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green
  strip.show();
  delay(500);
  strip.setPixelColor(0, strip.Color(0, 0, 255)); // Blue
  strip.show();
  delay(500);
  strip.setPixelColor(0, strip.Color(0, 0, 0)); // Off
  strip.show();
  
  // Initialize I2C and sensor (using your working setup)
  Wire.begin();
  sensor.init();
  
  // Test sensor
  Serial.print("Sensor test: ");
  Serial.println(sensor.getAngle());
  
  // SimpleFOC Mini driver setup
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;
  driver.init();
  Serial.println("SimpleFOC Mini driver initialized");
  
  // Link driver and sensor to motor
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  
  // Start with velocity control mode
  motor.controller = MotionControlType::velocity;
  
  // Settings for good performance
  motor.voltage_limit = 6;
  motor.velocity_limit = 25;
  
  // Velocity PID tuning
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 500;
  
  // Position PID tuning (for angle control)
  motor.P_angle.P = 15;     // Position proportional gain
  motor.P_angle.I = 0;      // No integral for position
  motor.P_angle.D = 0;      // No derivative for position
  
  // Velocity filtering
  motor.LPF_velocity.Tf = 0.02;
  
  motor.init();
  
  Serial.println("Starting FOC calibration...");
  motor.initFOC();
  
  Serial.println("Motor ready!");
  Serial.println("Commands:");
  Serial.println("  V<value> - Velocity mode (e.g., 'V2')");
  Serial.println("  P<degrees> - Position mode (e.g., 'P90', 'P180')");
  Serial.println("  M - Switch to velocity mode");
  Serial.println("  A - Switch to angle mode");
  
  // Set initial LED status
  updateLED();
}

void loop() {
  motor.loopFOC();
  
  if (Serial.available()) {
    String input = Serial.readString();
    input.trim();
    input.toUpperCase();
    
    if (input.startsWith("V")) {
      // Velocity command
      float target = input.substring(1).toFloat();
      if (motor.controller != MotionControlType::velocity) {
        motor.controller = MotionControlType::velocity;
        Serial.println("Switched to velocity mode");
        updateLED();
      }
      motor.move(target);
      Serial.print("Setting velocity: ");
      Serial.println(target);
      
    } else if (input.startsWith("P")) {
      // Position command in degrees
      float degrees = input.substring(1).toFloat();
      float radians = degrees * PI / 180.0;
      if (motor.controller != MotionControlType::angle) {
        motor.controller = MotionControlType::angle;
        Serial.println("Switched to position mode");
        updateLED();
      }
      motor.move(radians);
      Serial.print("Setting position: ");
      Serial.print(degrees);
      Serial.println("°");
      
    } else if (input == "M") {
      // Switch to velocity mode
      motor.controller = MotionControlType::velocity;
      Serial.println("Switched to velocity mode");
      updateLED();
      
    } else if (input == "A") {
      // Switch to angle mode
      motor.controller = MotionControlType::angle;
      Serial.println("Switched to position mode");
      updateLED();
      
    } else {
      // Legacy: just numbers for velocity
      float target = input.toFloat();
      if (target != 0 || input == "0") {
        motor.move(target);
        Serial.print("Setting velocity: ");
        Serial.println(target);
      }
    }
  }
  
  motor.move();
  
  // Status every second
  static unsigned long last_print = 0;
  if (millis() - last_print > 1000) {
    float angle_deg = sensor.getAngle() * 180.0 / PI;
    Serial.print("Angle: ");
    Serial.print(angle_deg, 1);
    Serial.print("° | Vel: ");
    Serial.print(sensor.getVelocity(), 2);
    Serial.print(" | Mode: ");
    Serial.print(motor.controller == MotionControlType::velocity ? "VEL" : "POS");
    Serial.print(" | Target: ");
    if (motor.controller == MotionControlType::angle) {
      Serial.print(motor.target * 180.0 / PI, 1);
      Serial.print("°");
    } else {
      Serial.print(motor.target);
    }
    Serial.print(" | V: ");
    Serial.println(motor.voltage.q, 2);
    last_print = millis();
  }
}