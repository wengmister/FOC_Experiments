#include <Arduino.h>
#include <SimpleFOC.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 33
#define NEOPIXEL_POWER 21
#define NUM_LEDS 1
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Single motor instance (7 pole pairs)
BLDCMotor motor = BLDCMotor(7);

// SimpleFOC Mini driver - EN=A5, IN1/2/3=9/6/5
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 6, 5, A5);

// I2C magnetic sensor
MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0x0E, 4);  // Wire (SDA=3, SCL=4 - default)

// Ratchet wheel variables
float last_angle = 0.0;
bool ratchet_enabled = true;


// Control loop timing
unsigned long last_control_update = 0;
const unsigned long control_interval = 5;  // Update every 5ms instead of every loop

// LED status function
void updateLED() {
  if (ratchet_enabled) {
    // Ratchet mode = Green
    strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green
    Serial.println("LED: Ratchet mode (GREEN)");
  } else {
    // Show different colors based on motor mode
    if (motor.controller == MotionControlType::velocity) {
      // Velocity mode = Red
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red
      Serial.println("LED: Velocity mode (RED)");
    } else if (motor.controller == MotionControlType::angle) {
      // Position mode = Blue
      strip.setPixelColor(0, strip.Color(0, 0, 255)); // Blue
      Serial.println("LED: Position mode (BLUE)");
    } else {
      // Torque mode = Purple
      strip.setPixelColor(0, strip.Color(255, 0, 255)); // Purple
      Serial.println("LED: Torque mode (PURPLE)");
    }
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
  Serial.println("LED: Green=Ratchet, Blue=Free");
  
  // Test NeoPixel with victory colors!
  Serial.println("NeoPixel test!");
  strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green
  strip.show();
  delay(500);
  strip.setPixelColor(0, strip.Color(0, 0, 0)); // Off
  strip.show();
  
  // Initialize I2C bus for sensor (ESP32-S3 Feather pins)
  Wire.begin(3, 4);      // I2C bus 0 - default SDA=3, SCL=4
  
  // Initialize sensor
  sensor.init(&Wire);
  
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
  
  // Start with torque control mode for ratchet
  motor.controller = MotionControlType::torque;
  
  // Motor settings for ratchet operation
  motor.voltage_limit = 6;
  motor.velocity_limit = 25;
  
  // Velocity PID tuning
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 500;
  
  // Position PID tuning (for angle control) - improved for tighter control
  motor.P_angle.P = 20;     // Higher proportional gain for better tracking
  motor.P_angle.I = 0.1;    // Small integral to eliminate steady state error
  motor.P_angle.D = 0.05;   // Small derivative for damping
  
  // Velocity filtering
  motor.LPF_velocity.Tf = 0.02;
  
  motor.init();
  
  Serial.println("Starting FOC calibration...");
  motor.initFOC();
  
  Serial.println("Motor ready!");
  last_angle = sensor.getAngle();
  Serial.println("Commands:");
  Serial.println("  R - Toggle ratchet mode on/off");
  Serial.println("  V<value> - Set velocity (e.g., 'V2')");
  Serial.println("  P<degrees> - Set position (e.g., 'P90')");
  Serial.println("  M - Switch to velocity mode");
  Serial.println("  A - Switch to angle mode");
  Serial.println("  T - Switch to torque mode (for ratchet)");
  Serial.println("  Z - Reset angle to zero (fixes position control)");
  
  // Set initial LED status
  updateLED();
}

void loop() {
  motor.loopFOC();
  
  if (Serial.available()) {
    String input = Serial.readString();
    input.trim();
    input.toUpperCase();
    
    if (input == "R") {
      ratchet_enabled = !ratchet_enabled;
      Serial.print("Ratchet mode: ");
      Serial.println(ratchet_enabled ? "ENABLED" : "DISABLED");
      if (ratchet_enabled) {
        motor.controller = MotionControlType::torque;
        last_angle = sensor.getAngle();
      } else {
        motor.controller = MotionControlType::velocity;
        motor.move(0);
      }
      updateLED();
      
    } else if (input.startsWith("V")) {
      float target = input.substring(1).toFloat();
      if (motor.controller != MotionControlType::velocity) {
        motor.controller = MotionControlType::velocity;
        ratchet_enabled = false;
        Serial.println("Motor switched to velocity mode");
        updateLED();
      }
      motor.move(target);
      Serial.print("Motor velocity: ");
      Serial.println(target);
      
    } else if (input.startsWith("P")) {
      float degrees = input.substring(1).toFloat();
      float radians = degrees * PI / 180.0;
      if (motor.controller != MotionControlType::angle) {
        motor.controller = MotionControlType::angle;
        ratchet_enabled = false;
        Serial.println("Motor switched to position mode");
        updateLED();
      }
      motor.move(radians);
      Serial.print("Motor position: ");
      Serial.print(degrees);
      Serial.println("°");
      
    } else if (input == "M") {
      motor.controller = MotionControlType::velocity;
      ratchet_enabled = false;
      Serial.println("Motor switched to velocity mode");
      updateLED();
      
    } else if (input == "A") {
      motor.controller = MotionControlType::angle;
      ratchet_enabled = false;
      Serial.println("Motor switched to position mode");
      updateLED();
      
    } else if (input == "T") {
      motor.controller = MotionControlType::torque;
      ratchet_enabled = true;
      last_angle = sensor.getAngle();
      Serial.println("Motor switched to torque mode (ratchet)");
      updateLED();
      
    } else if (input == "Z") {
      // Reset motor shaft angle to zero
      motor.shaft_angle = 0;
      Serial.print("Motor shaft angle reset to zero. Current sensor reading: ");
      Serial.print(sensor.getAngle() * 180.0 / PI, 1);
      Serial.println("°");
    }
  }
  
  // Special control modes
  if (motor.controller == MotionControlType::torque && ratchet_enabled && millis() - last_control_update > control_interval) {
    // Ratchet control logic - very gentle
    float velocity = sensor.getVelocity();
    float torque = 0;
    
    // Apply very light resistance based on velocity direction
    if (velocity > 0.5) {
      // Clockwise rotation - very light resistance
      torque = -0.01 * velocity;
    } else if (velocity < -0.5) {
      // Counter-clockwise rotation - stronger but still gentle resistance
      torque = -0.25 * velocity;
    }
    // No torque for small velocities to prevent drift
    
    motor.move(torque);
    last_control_update = millis();
    
  } else if (motor.controller != MotionControlType::torque) {
    // Normal mode - update motor (includes angle control)
    motor.move();
  }
  
  // Status every second
  static unsigned long last_print = 0;
  if (millis() - last_print > 1000) {
    float angle_deg = sensor.getAngle() * 180.0 / PI;
    Serial.print("Angle: ");
    Serial.print(angle_deg, 1);
    Serial.print("° | Vel: ");
    Serial.print(sensor.getVelocity(), 1);
    Serial.print(" | Mode: ");
    
    if (ratchet_enabled) {
      Serial.print("RATCHET");
    } else {
      if (motor.controller == MotionControlType::velocity) {
        Serial.print("VEL");
      } else if (motor.controller == MotionControlType::angle) {
        Serial.print("POS");
      } else {
        Serial.print("TOR");
      }
    }
    
    Serial.print(" | Target: ");
    if (motor.controller == MotionControlType::angle) {
      Serial.print(motor.target * 180.0 / PI, 1);
      Serial.print("°");
    } else {
      Serial.print(motor.target, 1);
    }
    Serial.print(" | V: ");
    Serial.print(motor.voltage.q, 1);
    Serial.println();
    
    last_print = millis();
  }
}