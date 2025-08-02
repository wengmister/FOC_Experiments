#include <Arduino.h>
#include <SimpleFOC.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 33
#define NEOPIXEL_POWER 21
#define NUM_LEDS 1
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Motor 1 instance (7 pole pairs) - Yaw axis
BLDCMotor motor1 = BLDCMotor(7);
// Motor 2 instance (7 pole pairs) - Pitch axis  
BLDCMotor motor2 = BLDCMotor(7);

// SimpleFOC Mini driver 1 - EN=A5, IN1/2/3=9/6/5
BLDCDriver3PWM driver1 = BLDCDriver3PWM(9, 6, 5, A5);
// SimpleFOC Mini driver 2 - EN=A4, IN1/2/3=10/11/12
BLDCDriver3PWM driver2 = BLDCDriver3PWM(12, 11, 10, A4);

// I2C magnetic sensors - separate buses
MagneticSensorI2C sensor1 = MagneticSensorI2C(0x36, 12, 0x0E, 4);  // Wire (SDA=3, SCL=4 - default)
MagneticSensorI2C sensor2 = MagneticSensorI2C(0x36, 12, 0x0E, 4);  // Wire1 (SDA=A0, SCL=A1)

// Master-slave mode variables
bool master_slave_mode = false;
float slave_scale = 1.0;  // 1:1 mapping by default
float master_offset = 0.0;  // Store initial master position when mode enabled

// LED status function with WORKING NeoPixel!
void updateLED() {
  if (master_slave_mode) {
    // Master-slave mode = Yellow
    strip.setPixelColor(0, strip.Color(255, 255, 0)); // Yellow
    Serial.println("LED: Master-slave mode (YELLOW)");
  } else {
    // Show different colors based on motor modes
    bool motor1_vel = (motor1.controller == MotionControlType::velocity);
    bool motor2_vel = (motor2.controller == MotionControlType::velocity);
    
    if (motor1_vel && motor2_vel) {
      // Both velocity = Red
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red
      Serial.println("LED: Both motors velocity mode (RED)");
    } else if (!motor1_vel && !motor2_vel) {
      // Both position = Blue
      strip.setPixelColor(0, strip.Color(0, 0, 255)); // Blue
      Serial.println("LED: Both motors position mode (BLUE)");
    } else {
      // Mixed mode = Purple
      strip.setPixelColor(0, strip.Color(255, 0, 255)); // Purple
      Serial.println("LED: Mixed motor modes (PURPLE)");
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
  Serial.println("LED: Red=Velocity, Blue=Position");
  
  // Test NeoPixel with victory colors!
  Serial.println("NeoPixel test!");
  strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green
  strip.show();
  delay(500);
  strip.setPixelColor(0, strip.Color(0, 0, 0)); // Off
  strip.show();
  
  // Initialize I2C buses for dual sensors (ESP32-S3 Feather pins)
  Wire.begin(3, 4);      // I2C bus 0 - default SDA=3, SCL=4
  Wire1.begin(A0, A1);   // I2C bus 1 - SDA=A0, SCL=A1
  
  // Initialize sensors on separate buses
  sensor1.init(&Wire);
  sensor2.init(&Wire1);
  
  // Test sensors
  Serial.print("Sensor 1 test: ");
  Serial.println(sensor1.getAngle());
  Serial.print("Sensor 2 test: ");
  Serial.println(sensor2.getAngle());
  
  // SimpleFOC Mini driver setup
  driver1.voltage_power_supply = 12;
  driver1.voltage_limit = 6;
  driver1.init();
  Serial.println("SimpleFOC Mini driver 1 initialized");
  
  driver2.voltage_power_supply = 12;
  driver2.voltage_limit = 6;
  driver2.init();
  Serial.println("SimpleFOC Mini driver 2 initialized");
  
  // Link drivers and sensors to motors
  motor1.linkDriver(&driver1);
  motor1.linkSensor(&sensor1);
  motor2.linkDriver(&driver2);
  motor2.linkSensor(&sensor2);
  
  // Start with velocity control mode
  motor1.controller = MotionControlType::velocity;
  motor2.controller = MotionControlType::velocity;
  
  // Settings for good performance - Motor 1 (Yaw)
  motor1.voltage_limit = 6;
  motor1.velocity_limit = 25;
  
  // Velocity PID tuning - Motor 1
  motor1.PID_velocity.P = 0.1;
  motor1.PID_velocity.I = 1.0;
  motor1.PID_velocity.D = 0.0;
  motor1.PID_velocity.output_ramp = 500;
  
  // Position PID tuning (for angle control) - Motor 1
  motor1.P_angle.P = 15;     // Position proportional gain
  motor1.P_angle.I = 0;      // No integral for position
  motor1.P_angle.D = 0;      // No derivative for position
  
  // Velocity filtering - Motor 1
  motor1.LPF_velocity.Tf = 0.02;
  
  // Settings for good performance - Motor 2 (Pitch)
  motor2.voltage_limit = 6;
  motor2.velocity_limit = 25;
  
  // Velocity PID tuning - Motor 2
  motor2.PID_velocity.P = 0.1;
  motor2.PID_velocity.I = 1.0;
  motor2.PID_velocity.D = 0.0;
  motor2.PID_velocity.output_ramp = 500;
  
  // Position PID tuning (for angle control) - Motor 2
  motor2.P_angle.P = 15;     // Position proportional gain
  motor2.P_angle.I = 0;      // No integral for position
  motor2.P_angle.D = 0;      // No derivative for position
  
  // Velocity filtering - Motor 2
  motor2.LPF_velocity.Tf = 0.02;
  
  motor1.init();
  motor2.init();
  
  Serial.println("Starting FOC calibration...");
  motor1.initFOC();
  motor2.initFOC();
  
  Serial.println("Both motors ready!");
  Serial.println("Commands:");
  Serial.println("  1V<value> - Motor 1 velocity (e.g., '1V2')");
  Serial.println("  2V<value> - Motor 2 velocity (e.g., '2V2')");
  Serial.println("  1P<degrees> - Motor 1 position (e.g., '1P90')");
  Serial.println("  2P<degrees> - Motor 2 position (e.g., '2P180')");
  Serial.println("  V<value> - Both motors velocity (e.g., 'V2')");
  Serial.println("  P<degrees> - Both motors position (e.g., 'P90')");
  Serial.println("  1M/2M - Switch motor to velocity mode");
  Serial.println("  1A/2A - Switch motor to angle mode");
  Serial.println("  M/A - Switch both motors to velocity/angle mode");
  Serial.println("  S - Toggle master-slave mode (Motor1 -> Motor2)");
  Serial.println("  SC<ratio> - Set slave scale ratio (e.g., 'SC2' for 2:1)");
  
  // Set initial LED status
  updateLED();
}

void loop() {
  motor1.loopFOC();
  motor2.loopFOC();
  
  if (Serial.available()) {
    String input = Serial.readString();
    input.trim();
    input.toUpperCase();
    
    // Motor 1 specific commands
    if (input.startsWith("1V")) {
      float target = input.substring(2).toFloat();
      if (motor1.controller != MotionControlType::velocity) {
        motor1.controller = MotionControlType::velocity;
        Serial.println("Motor 1 switched to velocity mode");
        updateLED();
      }
      motor1.move(target);
      Serial.print("Motor 1 velocity: ");
      Serial.println(target);
      
    } else if (input.startsWith("1P")) {
      float degrees = input.substring(2).toFloat();
      float radians = degrees * PI / 180.0;
      if (motor1.controller != MotionControlType::angle) {
        motor1.controller = MotionControlType::angle;
        Serial.println("Motor 1 switched to position mode");
        updateLED();
      }
      motor1.move(radians);
      Serial.print("Motor 1 position: ");
      Serial.print(degrees);
      Serial.println("°");
      
    } else if (input == "1M") {
      motor1.controller = MotionControlType::velocity;
      Serial.println("Motor 1 switched to velocity mode");
      updateLED();
      
    } else if (input == "1A") {
      motor1.controller = MotionControlType::angle;
      Serial.println("Motor 1 switched to position mode");
      updateLED();
      
    // Motor 2 specific commands  
    } else if (input.startsWith("2V")) {
      float target = input.substring(2).toFloat();
      if (motor2.controller != MotionControlType::velocity) {
        motor2.controller = MotionControlType::velocity;
        Serial.println("Motor 2 switched to velocity mode");
        updateLED();
      }
      motor2.move(target);
      Serial.print("Motor 2 velocity: ");
      Serial.println(target);
      
    } else if (input.startsWith("2P")) {
      float degrees = input.substring(2).toFloat();
      float radians = degrees * PI / 180.0;
      if (motor2.controller != MotionControlType::angle) {
        motor2.controller = MotionControlType::angle;
        Serial.println("Motor 2 switched to position mode");
        updateLED();
      }
      motor2.move(radians);
      Serial.print("Motor 2 position: ");
      Serial.print(degrees);
      Serial.println("°");
      
    } else if (input == "2M") {
      motor2.controller = MotionControlType::velocity;
      Serial.println("Motor 2 switched to velocity mode");
      updateLED();
      
    } else if (input == "2A") {
      motor2.controller = MotionControlType::angle;
      Serial.println("Motor 2 switched to position mode");
      updateLED();
      
    // Both motors commands
    } else if (input.startsWith("V")) {
      float target = input.substring(1).toFloat();
      if (motor1.controller != MotionControlType::velocity) {
        motor1.controller = MotionControlType::velocity;
        Serial.println("Both motors switched to velocity mode");
        updateLED();
      }
      if (motor2.controller != MotionControlType::velocity) {
        motor2.controller = MotionControlType::velocity;
        if (motor1.controller == MotionControlType::velocity) {
          Serial.println("Both motors switched to velocity mode");
          updateLED();
        }
      }
      motor1.move(target);
      motor2.move(target);
      Serial.print("Both motors velocity: ");
      Serial.println(target);
      
    } else if (input.startsWith("P")) {
      float degrees = input.substring(1).toFloat();
      float radians = degrees * PI / 180.0;
      if (motor1.controller != MotionControlType::angle || motor2.controller != MotionControlType::angle) {
        motor1.controller = MotionControlType::angle;
        motor2.controller = MotionControlType::angle;
        Serial.println("Both motors switched to position mode");
        updateLED();
      }
      motor1.move(radians);
      motor2.move(radians);
      Serial.print("Both motors position: ");
      Serial.print(degrees);
      Serial.println("°");
      
    } else if (input == "M") {
      // Exit master-slave mode if active
      if (master_slave_mode) {
        master_slave_mode = false;
        Serial.println("Master-slave mode auto-disabled");
      }
      motor1.controller = MotionControlType::velocity;
      motor2.controller = MotionControlType::velocity;
      Serial.println("Both motors switched to velocity mode");
      updateLED();
      
    } else if (input == "A") {
      // Exit master-slave mode if active
      if (master_slave_mode) {
        master_slave_mode = false;
        Serial.println("Master-slave mode auto-disabled");
      }
      motor1.controller = MotionControlType::angle;
      motor2.controller = MotionControlType::angle;
      Serial.println("Both motors switched to position mode");
      updateLED();
      
    // Master-slave mode commands
    } else if (input == "S") {
      master_slave_mode = !master_slave_mode;
      Serial.print("DEBUG: master_slave_mode = ");
      Serial.println(master_slave_mode ? "true" : "false");
      
      if (master_slave_mode) {
        // Enable master-slave mode
        master_offset = sensor1.getAngle();  // Store current master position as zero
        motor1.controller = MotionControlType::torque;  // Master: free to move (torque mode)
        motor2.controller = MotionControlType::angle;   // Slave: position control
        motor1.move(0);  // No torque on master - free to turn
        Serial.print("Master-slave mode ENABLED. Scale: ");
        Serial.print(slave_scale);
        Serial.println(":1");
        Serial.println("Motor1 (master/torque) -> Motor2 (slave/position)");
      } else {
        // Return to normal operation
        motor1.controller = MotionControlType::angle;
        motor2.controller = MotionControlType::angle;
        Serial.println("Master-slave mode DISABLED");
      }
      updateLED();
      
    } else if (input.startsWith("SC")) {
      float scale = input.substring(2).toFloat();
      if (scale > 0.1 && scale <= 10.0) {  // Reasonable scale limits
        slave_scale = scale;
        Serial.print("Slave scale set to ");
        Serial.print(slave_scale);
        Serial.println(":1");
      } else {
        Serial.println("Scale must be between 0.1 and 10.0");
      }
    }
  }
  
  // Master-slave mode control
  if (master_slave_mode) {
    // Read master motor position (Motor 1 in torque mode - free to move)
    float master_angle = sensor1.getAngle();
    
    // Calculate relative movement from initial position
    float relative_angle = (master_angle - master_offset) * slave_scale;
    
    // Set slave motor target (Motor 2 in position mode)
    motor2.move(relative_angle);
    
    // Keep master motor at zero torque (free to move)
    motor1.move(0);
  } else {
    // Normal mode - update both motors
    motor1.move();
    motor2.move();
  }
  
  // Status every second
  static unsigned long last_print = 0;
  if (millis() - last_print > 1000) {
    // Motor 1 status
    float angle1_deg = sensor1.getAngle() * 180.0 / PI;
    Serial.print("M1: ");
    Serial.print(angle1_deg, 1);
    Serial.print("° | ");
    Serial.print(sensor1.getVelocity(), 1);
    Serial.print(" | ");
    if (master_slave_mode) {
      Serial.print("MASTER");
    } else {
      Serial.print(motor1.controller == MotionControlType::velocity ? "VEL" : "POS");
    }
    Serial.print(" | T:");
    if (motor1.controller == MotionControlType::angle) {
      Serial.print(motor1.target * 180.0 / PI, 1);
      Serial.print("°");
    } else {
      Serial.print(motor1.target, 1);
    }
    Serial.print(" | V:");
    Serial.print(motor1.voltage.q, 1);
    
    // Motor 2 status
    float angle2_deg = sensor2.getAngle() * 180.0 / PI;
    Serial.print(" || M2: ");
    Serial.print(angle2_deg, 1);
    Serial.print("° | ");
    Serial.print(sensor2.getVelocity(), 1);
    Serial.print(" | ");
    if (master_slave_mode) {
      Serial.print("SLAVE");
    } else {
      Serial.print(motor2.controller == MotionControlType::velocity ? "VEL" : "POS");
    }
    Serial.print(" | T:");
    if (motor2.controller == MotionControlType::angle) {
      Serial.print(motor2.target * 180.0 / PI, 1);
      Serial.print("°");
    } else {
      Serial.print(motor2.target, 1);
    }
    Serial.print(" | V:");
    Serial.print(motor2.voltage.q, 1);
    
    // Show master-slave info
    if (master_slave_mode) {
      Serial.print(" | Scale:");
      Serial.print(slave_scale);
      Serial.print(":1");
    }
    Serial.println();
    
    last_print = millis();
  }
}