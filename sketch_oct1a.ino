#include <Wire.h>
#include <MPU6050.h>
// Add with other pin definitions
#define LED_PIN PC13  // Built-in LED on Black Pill (same as button)
//#include <IWatchdog.h>

// Add at top of file, outside any function
extern "C" {
    void HardFault_Handler(void) {
        Serial.println("HARD FAULT! System crashed.");
        while(1) {
            // Rapid LED blinking to indicate crash
            digitalWrite(LED_PIN, LOW);
            delay(100);
            digitalWrite(LED_PIN, HIGH);
            delay(100);
        }
    }
}


// Motor Driver Pins
#define ENA PA9 //PA9
#define ENB PA8
#define IN1 PB4
#define IN2 PB3
#define IN3 PB1
#define IN4 PB0

// MPU6050 I2C Pins
#define SDA PB7
#define SCL PB6

// MPU6050 object
MPU6050 mpu;

// PID Constants - Start with these values
float Kp = 8.0;    // Proportional gain
float Ki = 10;     // Integral gain (start with 0)
float Kd = 2.8;     // Derivative gain

// Directional PID - Different gains for forward vs backward tilt
float KpForward = 15.0;    // Kp when robot tilts forward (needs correction backward)
float KpBackward = 15.0;   // Kp when robot tilts backward (needs correction forward)
float KdForward = 3;     // Kd for forward tilt
float KdBackward = 3;    // Kd for backward tilt
bool useDirectionalPID = true;  // Enable/disable directional control

// PID variables
float setpoint = 0.0;       // Target angle (0 degrees = balanced)
float input, output;
float error, lastError = 0;
float integral = 0;
float derivative;
// Motor speed limits (40% of 255)
const int MAX_SPEED = 120;  // 40% of 255
const int MIN_SPEED = 15;   // Minimum speed to overcome friction

// Timing variables
unsigned long lastTime = 0;
unsigned long currentTime;
float deltaTime;

// Sensor variables
float pitch = 0;
float pitchAcc, pitchGyro;
float alpha = 0.98; // Complementary filter constant

// Safety and control flags
bool balanceMode = false;
bool motorEnabled = true;
unsigned long lastStatusPrint = 0;
bool motorReversed = false;  // Flag to reverse motor direction if needed

// Safety monitoring
unsigned long lastSafetyCheck = 0;
unsigned long lastLoopTime = 0;
float maxAllowedOutput = MAX_SPEED * 0.8;  // 80% of max speed as safety limit
bool emergencyStop = false;
unsigned long motorRunTime = 0;
unsigned long lastMotorStart = 0;
bool enableTimingCheck = false;  // Disable timing check during startup/calibration

// Auto-tuning variables
bool autoTune = false;
float tuneStep = 0.5;
unsigned long tuneTimer = 0;
int tunePhase = 0;

// Calibration variables
bool isCalibrating = false;
float pitchOffset = 0.0;  // Manual pitch offset for fine-tuning balance point

void setup() {
  Serial.begin(115200);
  delay(1000);

   // Check if we're restarting after crash
    // if (IWatchdog.isReset()) {
    //     Serial.println("RECOVERY: Watchdog reset detected - system was frozen!");
    //     // Blink LED rapidly to indicate recovery
    //     for (int i = 0; i < 10; i++) {
    //         digitalWrite(LED_PIN, LOW);
    //         delay(100);
    //         digitalWrite(LED_PIN, HIGH);
    //         delay(100);
    //     }
    // }
  
  Serial.println("=== Self-Balancing Robot with Easy PID Tuning ===");
  Serial.println("Type 'help' for commands");
  
  // Initialize motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();
  
  // Initialize I2C
  Wire.setSDA(SDA);
  Wire.setSCL(SCL);
  Wire.begin();
  Wire.setClock(400000); // 400kHz for better performance
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully!");
    
    // Quick calibration
    Serial.println("Calibrating... Keep robot upright and still!");
    delay(3000);
    calibrateMPU();
    
  } else {
    Serial.println("MPU6050 connection failed!");
    while(1);
  }
  
  // Initialize timing
  lastTime = millis();
  
  // Initialize pitch angle directly from accelerometer first
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  
  // Set initial pitch from accelerometer
  pitch = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI + pitchOffset;
  
  Serial.println("Stabilizing angle reading...");
  Serial.println("Initial accelerometer reading: " + String(pitch, 1) + "°");
  
  // Reset timing before stabilization
  lastLoopTime = millis();
  
  // Now stabilize with complementary filter
  for (int i = 0; i < 100; i++) {
    readAndProcessSensor();
    delay(10);
    lastLoopTime = millis(); // Update timing during stabilization
  }
  
  // Now set the target to the stabilized angle
  setpoint = pitch;
  
  // Enable timing check after initialization is complete
  lastLoopTime = millis();
  enableTimingCheck = true;

  pinMode(LED_PIN, OUTPUT);
digitalWrite(LED_PIN, HIGH);
  
  Serial.println("\n=== Ready to Balance ===");
  Serial.println("Stabilized angle: " + String(pitch, 1) + "°");
  Serial.println("Target automatically set to: " + String(setpoint, 1) + "°");
  Serial.println("Commands: 'start' to begin, 'help' for all commands");
  Serial.println("Current PID: P=" + String(Kp) + " I=" + String(Ki) + " D=" + String(Kd));


// if (IWatchdog.isReset()) {
//     Serial.println("WATCHDOG: System recovered from crash!");
//     }

//     IWatchdog.begin(800000); // 800ms watchdog

 }

void loop() {
 // IWatchdog.reload();
 static unsigned long lastBlink = 0;
if (millis() - lastBlink > 200) {  // Blink every 500ms
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Toggle
    lastBlink = millis();
}


  currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;
  
  // Handle serial commands
  handleSerialCommands();
  
  // Main control loop - run at ~200Hz for stability
  if (deltaTime >= 0.005) {
      if (deltaTime > 1.0) {  // If more than 1 second passed
        deltaTime = 0.005;  // Reset to minimum
        lastError = 0;      // Reset PID memory
        integral = 0;
    }
    // Safety checks first
    if (!performSafetyChecks()) {
      stopMotorsEmergency();
      return;
    }
    
    // Read and process sensor data
    readAndProcessSensor();
    
    // Balance control
    if (balanceMode && motorEnabled && !emergencyStop) {
      calculatePID();
      
      // Additional output safety limit
      output = constrain(output, -maxAllowedOutput, maxAllowedOutput);
      
      controlMotors(output);
      
      // Safety check - stop if too tilted
      if (abs(pitch - setpoint) > 30) {
        balanceMode = false;
        stopMotors();
        Serial.println("Safety stop - robot too far from target!");
      }
    } else {
      stopMotors();
    }
    
    lastTime = currentTime;
    lastLoopTime = millis(); // Track loop timing for watchdog
  }
  
  // Status printing every 200ms
  if (currentTime - lastStatusPrint > 500 && balanceMode) {
    printStatus();
    lastStatusPrint = currentTime;
  }
  
  // Auto-tuning logic
  if (autoTune) {
    performAutoTune();
  }
}

void readAndProcessSensor() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert to meaningful units
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  float gyroY = gy / 131.0; // Pitch rate
  
  // Calculate pitch from accelerometer
  pitchAcc = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  
  // Integrate gyro for pitch
  pitchGyro = pitch + gyroY * deltaTime;
  
  // Complementary filter with offset compensation
  pitch = alpha * pitchGyro + (1 - alpha) * pitchAcc + pitchOffset;
}

void calculatePID() {

    // Add sanity checks at start
    if (isnan(pitch) || isinf(pitch)) {
        Serial.println("PID: Invalid pitch, resetting!");
        pitch = setpoint;
        return;
    }


  input = pitch;
  error = setpoint - input;
  
  // Determine current Kp and Kd based on tilt direction (if directional PID enabled)
  float currentKp = Kp;
  float currentKd = Kd;
  
  if (useDirectionalPID) {
    if (error > 0) {
      // Robot tilted backward (needs forward correction)
      currentKp = KpBackward;
      currentKd = KdBackward;
    } else {
      // Robot tilted forward (needs backward correction)  
      currentKp = KpForward;
      currentKd = KdForward;
    }
  }
  
  // Proportional term
  float P = currentKp * error;
  
  // Integral term with windup protection
  integral += error * deltaTime;
  integral = constrain(integral, -5, 5);
  float I = Ki * integral;
  
  // Derivative term with smoothing
  // derivative = (error - lastError) / deltaTime;
  // float D = currentKd * derivative;

   if (deltaTime > 0.001f) { // Ensure reasonable deltaTime
        derivative = (error - lastError) / deltaTime;
        // Prevent derivative windup
        derivative = constrain(derivative, -1000.0f, 1000.0f);
    } else {
        derivative = 0.0f;
    }

      // Check for NaN/Inf in calculations
    if (isnan(derivative) || isinf(derivative)) {
        derivative = 0.0f;
    }
    
    float D = currentKd * derivative;
  
  // PID output
  output = P + I + D;
  
  // Update for next iteration
  lastError = error;
}

void controlMotors(float speed) {
  // Emergency stop check
  if (emergencyStop) {
    stopMotorsEmergency();
    return;
  }
  
  // Apply speed limits with extra safety margin
  speed = constrain(speed, -maxAllowedOutput, maxAllowedOutput);
  
  // Sanity check - if speed is unreasonable, stop everything
  if (abs(speed) > MAX_SPEED || isnan(speed) || isinf(speed)) {
    Serial.println("EMERGENCY: Invalid motor speed detected!");
    emergencyStop = true;
    stopMotorsEmergency();
    return;
  }
  
  // Dead zone to prevent jitter
  if (abs(speed) < MIN_SPEED) {
    stopMotors();
    return;
  }
  
  // Track motor runtime for safety
  if (abs(speed) > MIN_SPEED) {
    if (lastMotorStart == 0) {
      lastMotorStart = millis();
    }
    motorRunTime = millis() - lastMotorStart;
    
    // Safety: Stop if motors run at high speed for too long
    if (abs(speed) > MAX_SPEED * 0.7 && motorRunTime > 3000) {
      Serial.println("EMERGENCY: Motors running too long at high speed!");
      emergencyStop = true;
      stopMotorsEmergency();
      return;
    }
  } else {
    lastMotorStart = 0;
    motorRunTime = 0;
  }
  
  // Reverse motor direction if flag is set
  if (motorReversed) {
    speed = -speed;
  }
  
  // Both motors move in same direction for balancing
  int motorSpeed = abs(speed);
  
  // Final safety check on PWM value
  motorSpeed = constrain(motorSpeed, 0, MAX_SPEED);
  
  if (speed > 0) {
    // Move forward to correct backward tilt
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    // Move backward to correct forward tilt
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  lastMotorStart = 0;
  motorRunTime = 0;
}

// Emergency motor stop with status
void stopMotorsEmergency() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  balanceMode = false;
  lastMotorStart = 0;
  motorRunTime = 0;
  
  Serial.println("EMERGENCY STOP ACTIVATED!");
  Serial.println("Motors forcibly stopped. Use 'clearemergency' to reset.");
}

void calibrateMPU() {
  // Simple offset calibration
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;
  
  for (int i = 0; i < 1000; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    axSum += ax;
    aySum += ay;
    azSum += az;
    gxSum += gx;
    gySum += gy;
    gzSum += gz;
    
    delay(2);
  }
  
  // Set offsets (simplified)
  mpu.setXAccelOffset(-(axSum / 1000) / 8);
  mpu.setYAccelOffset(-(aySum / 1000) / 8);
  mpu.setZAccelOffset((16384 - (azSum / 1000)) / 8);
  mpu.setXGyroOffset(-(gxSum / 1000) / 4);
  mpu.setYGyroOffset(-(gySum / 1000) / 4);
  mpu.setZGyroOffset(-(gzSum / 1000) / 4);
  
  // Initialize pitch angle from accelerometer
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  
  // Calculate initial pitch from accelerometer
  pitch = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI + pitchOffset;
  
  Serial.println("Calibration complete!");
}

void handleSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readString();
    cmd.trim();
    cmd.toLowerCase();
    
    if (cmd == "start") {
      // Check if robot is close to target before starting
      readAndProcessSensor();
      float errorFromTarget = abs(pitch - setpoint);
      
      if (errorFromTarget > 35) {
        Serial.println("Cannot start - robot too far from target!");
        Serial.println("Current pitch: " + String(pitch, 1) + "° | Target: " + String(setpoint, 1) + "°");
        Serial.println("Error: " + String(errorFromTarget, 1) + "° (max allowed: 35°)");
        Serial.println("Use 'quickcal' to set current position as balance point");
        Serial.println("Or use 'target" + String((int)pitch) + "' to set target to current angle");
      } else {
        balanceMode = true;
        integral = 0; // Reset integral
        lastError = 0;
        Serial.println("Balance mode STARTED");
        Serial.println("Current: " + String(pitch, 1) + "° | Target: " + String(setpoint, 1) + "°");
      }
      
    } else if (cmd == "stop") {
      balanceMode = false;
      stopMotors();
      Serial.println("Balance mode STOPPED");
      
    } else if (cmd == "tune") {
      startTuningMode();
      
    } else if (cmd == "autotune") {
      autoTune = true;
      tuneTimer = millis();
      tunePhase = 0;
      Serial.println("Auto-tuning started...");
      
    } else if (cmd == "status") {
      printDetailedStatus();
      
    } else if (cmd == "recalibrate" || cmd == "cal") {
      recalibrateSensor();
      
    } else if (cmd == "quickcal") {
      quickCalibrate();
      
    } else if (cmd == "angle") {
      showCurrentAngle();
      
    } else if (cmd == "offset") {
      showPitchOffset();
      
    } else if (cmd == "autoset") {
      // Automatically set target to current angle
      readAndProcessSensor();
      setpoint = pitch;
      Serial.println("Target automatically set to current angle: " + String(setpoint, 1) + "°");
      
    } else if (cmd == "directional") {
      toggleDirectionalPID();
      
    } else if (cmd.startsWith("pf")) {
      KpForward = cmd.substring(2).toFloat();
      Serial.println("Kp Forward = " + String(KpForward));
      
    } else if (cmd.startsWith("pb")) {
      KpBackward = cmd.substring(2).toFloat();
      Serial.println("Kp Backward = " + String(KpBackward));
      
    } else if (cmd.startsWith("df")) {
      KdForward = cmd.substring(2).toFloat();
      Serial.println("Kd Forward = " + String(KdForward));
      
    } else if (cmd.startsWith("db")) {
      KdBackward = cmd.substring(2).toFloat();
      Serial.println("Kd Backward = " + String(KdBackward));
      
    } else if (cmd == "clearemergency") {
      clearEmergencyStop();
      
    } else if (cmd == "safetylimit") {
      String limitStr = cmd.substring(11);
      if (limitStr.length() > 0) {
        maxAllowedOutput = limitStr.toFloat();
        maxAllowedOutput = constrain(maxAllowedOutput, 10, MAX_SPEED);
        Serial.println("Safety limit set to: " + String(maxAllowedOutput));
      } else {
        Serial.println("Current safety limit: " + String(maxAllowedOutput));
      }
      
    } else if (cmd == "help") {
      printHelp();
      
    } else if (cmd.startsWith("p")) {
      Kp = cmd.substring(1).toFloat();
      Serial.println("Kp = " + String(Kp));
      
    } else if (cmd.startsWith("i")) {
      Ki = cmd.substring(1).toFloat();
      integral = 0; // Reset integral when Ki changes
      Serial.println("Ki = " + String(Ki));
      
    } else if (cmd.startsWith("d")) {
      Kd = cmd.substring(1).toFloat();
      Serial.println("Kd = " + String(Kd));
      
    } else if (cmd.startsWith("s")) {
      setpoint = cmd.substring(1).toFloat();
      Serial.println("Setpoint = " + String(setpoint));
      
    } else if (cmd.startsWith("target")) {
      String targetStr = cmd.substring(6);
      if (targetStr.length() > 0) {
        float newTarget = targetStr.toFloat();
        setpoint = newTarget;
        Serial.println("Target angle set to: " + String(setpoint) + "°");
      } else {
        Serial.println("Usage: target2.5 or target-1.0");
      }
      
    } else if (cmd.startsWith("t") && cmd.length() > 1) {
      String targetStr = cmd.substring(1);
      if (targetStr.length() > 0) {
        float newTarget = targetStr.toFloat();
        setpoint = newTarget;
        Serial.println("Target angle set to: " + String(setpoint) + "°");
      } else {
        Serial.println("Usage: t2.5 or t-1.0");
      }
      
    } else if (cmd.startsWith("o") && cmd.length() > 1 && !cmd.startsWith("offset")) {
      String offsetStr = cmd.substring(1);
      if (offsetStr.length() > 0) {
        float newOffset = offsetStr.toFloat();
        pitchOffset = newOffset;
        Serial.println("Pitch offset set to: " + String(pitchOffset) + "°");
      } else {
        Serial.println("Usage: o1.2 or o-0.8");
      }
      
    } else {
      Serial.println("Unknown command: '" + cmd + "'. Type 'help' for available commands.");
    }
  }
}

void printHelp() {
  Serial.println("\n=== Available Commands ===");
  Serial.println("start          - Start balancing (checks if close to target)");
  Serial.println("balance        - Quick start (sets target to current angle)");
  Serial.println("stop           - Stop balancing");
  Serial.println("clearemergency - Clear emergency stop condition");
  Serial.println("safetylimit50  - Set motor speed safety limit (default 80)");
  Serial.println("autoset        - Set target to current sensor reading");
  Serial.println("reverse        - Reverse motor direction for balancing");
  Serial.println("directional    - Toggle directional PID (different gains for each direction)");
  Serial.println("tune           - Interactive tuning mode");
  Serial.println("autotune       - Automatic PID tuning");
  Serial.println("status         - Show detailed status");
  Serial.println("reset          - Reset PID to defaults");
  Serial.println("help           - Show this help");
  Serial.println("\n=== Calibration ===");
  Serial.println("recalibrate    - Full sensor recalibration (robot upright)");
  Serial.println("quickcal       - Quick balance point calibration");
  Serial.println("angle          - Show current pitch angle");
  Serial.println("offset         - Show current pitch offset");
  Serial.println("\n=== Target Angle ===");
  Serial.println("target68       - Set target angle to 68° (current reading)");
  Serial.println("target45       - Set target angle to 45°");
  Serial.println("target-10      - Set target angle to -10°");
  Serial.println("t2.5           - Short: Set target to 2.5°");
  Serial.println("offset1.2      - Add 1.2° pitch offset");
  Serial.println("offset-0.8     - Add -0.8° pitch offset");
  Serial.println("o0.5           - Short: Add 0.5° offset");
  Serial.println("\n=== Standard PID Tuning ===");
  Serial.println("p15.0          - Set Kp to 15.0");
  Serial.println("i0.5           - Set Ki to 0.5");
  Serial.println("d1.2           - Set Kd to 1.2");
  Serial.println("s0.0           - Set setpoint to 0.0 (same as target0.0)");
  Serial.println("\n=== Directional PID (for asymmetric robots) ===");
  Serial.println("directional    - Enable/disable directional PID");
  Serial.println("pf25.0         - Set Kp for Forward tilt (needs more power)");
  Serial.println("pb15.0         - Set Kp for Backward tilt (needs less power)");
  Serial.println("df1.5          - Set Kd for Forward tilt");
  Serial.println("db0.8          - Set Kd for Backward tilt");
  Serial.println("\n=== Safety Features (for motor runaway issues) ===");
  Serial.println("1. Emergency stop activates if motors run too fast/long");
  Serial.println("2. Safety speed limit prevents full-speed runaway");
  Serial.println("3. Use 'clearemergency' if system locks up");
  Serial.println("4. 'safetylimit40' reduces max motor speed for testing");
  Serial.println("5. System monitors for infinite/NaN values");
}

void startTuningMode() {
  Serial.println("\n=== Interactive PID Tuning Mode ===");
  Serial.println("Current values: P=" + String(Kp) + " I=" + String(Ki) + " D=" + String(Kd));
  Serial.println("Pitch: " + String(pitch) + "° | Error: " + String(error));
  Serial.println("\nQuick adjustments:");
  Serial.println("P+/P- : Increase/decrease Kp by 1.0");
  Serial.println("I+/I- : Increase/decrease Ki by 0.1");
  Serial.println("D+/D- : Increase/decrease Kd by 0.1");
  Serial.println("Or use: p15.0, i0.5, d1.2 format");
}

void performAutoTune() {
  // Simple auto-tuning based on system response
  if (millis() - tuneTimer > 3000) { // Every 3 seconds
    if (tunePhase == 0) {
      // Phase 1: Find P that causes oscillation
      if (abs(pitch) < 2) {
        Kp += tuneStep;
        Serial.println("Auto-tune: Increasing Kp to " + String(Kp));
      } else {
        tunePhase = 1;
        Kp *= 0.8; // Reduce P by 20%
        Serial.println("Auto-tune: Oscillation found, reducing Kp to " + String(Kp));
      }
    } else if (tunePhase == 1) {
      // Phase 2: Add derivative
      Kd = Kp * 0.1;
      Serial.println("Auto-tune: Setting Kd to " + String(Kd));
      tunePhase = 2;
    } else {
      // Phase 3: Fine-tune integral
      if (abs(error) > 1) {
        Ki = Kp * 0.02;
        Serial.println("Auto-tune: Setting Ki to " + String(Ki));
      }
      autoTune = false;
      Serial.println("Auto-tune complete!");
    }
    tuneTimer = millis();
  }
}

void resetPID() {
  Kp = 15.0;
  Ki = 0.0;
  Kd = 0.8;
  setpoint = 0.0;
  integral = 0;
  pitchOffset = 0.0;
  Serial.println("PID reset to defaults");
}

void printStatus() {
  Serial.print("Pitch:");
  Serial.print(pitch, 1);
  Serial.print("° Err:");
  Serial.print(error, 1);
  Serial.print(" Out:");
  Serial.print(output, 0);
  
  if (useDirectionalPID) {
    if (error > 0) {
      Serial.print(" PB:");
      Serial.print(KpBackward);
      Serial.print(" DB:");
      Serial.print(KdBackward);
    } else {
      Serial.print(" PF:");
      Serial.print(KpForward);
      Serial.print(" DF:");
      Serial.print(KdForward);
    }
    Serial.print(" I:");
    Serial.println(Ki);
  } else {
    Serial.print(" P:");
    Serial.print(Kp);
    Serial.print(" I:");
    Serial.print(Ki);
    Serial.print(" D:");
    Serial.println(Kd);
  }
}

void printDetailedStatus() {
  Serial.println("\n=== Robot Status ===");
  Serial.println("Balance Mode: " + String(balanceMode ? "ON" : "OFF"));
  Serial.println("Raw Pitch: " + String(pitch - pitchOffset, 2) + "°");
  Serial.println("Corrected Pitch: " + String(pitch, 2) + "°");
  Serial.println("Pitch Offset: " + String(pitchOffset, 2) + "°");
  Serial.println("Target Angle: " + String(setpoint, 2) + "°");
  Serial.println("Error: " + String(error, 2) + "°");
  Serial.println("Motor Output: " + String(output, 1));
  Serial.println("Directional PID: " + String(useDirectionalPID ? "ENABLED" : "DISABLED"));
  
  if (useDirectionalPID) {
    Serial.println("=== Directional PID Values ===");
    Serial.println("Forward Tilt - Kp: " + String(KpForward) + " Kd: " + String(KdForward));
    Serial.println("Backward Tilt - Kp: " + String(KpBackward) + " Kd: " + String(KdBackward));
    Serial.println("Current Direction: " + String(error > 0 ? "BACKWARD (using Backward gains)" : "FORWARD (using Forward gains)"));
  } else {
    Serial.println("Standard PID Values: P=" + String(Kp) + " I=" + String(Ki) + " D=" + String(Kd));
  }
  
  Serial.println("Loop Time: " + String(deltaTime * 1000, 1) + "ms");
}

void recalibrateSensor() {
  if (balanceMode) {
    Serial.println("Please stop balancing mode first!");
    return;
  }
  
  Serial.println("\n=== Sensor Recalibration ===");
  Serial.println("Place robot in perfect upright position and keep still...");
  Serial.println("Calibration will start in 3 seconds...");
  
  for (int i = 3; i > 0; i--) {
    Serial.println(String(i) + "...");
    delay(1000);
  }
  
  Serial.println("Calibrating... Please keep robot still!");
  isCalibrating = true;
  
  calibrateMPU();
  pitchOffset = 0.0; // Reset pitch offset after full calibration
  
  isCalibrating = false;
  Serial.println("Sensor recalibration complete!");
  
  // Initialize pitch directly from accelerometer
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  
  pitch = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI + pitchOffset;
  
  Serial.println("Initial accelerometer reading: " + String(pitch, 2) + "°");
  
  // Reset timing before stabilization
  lastLoopTime = millis();
  
  // Stabilize angle reading after recalibration
  Serial.println("Stabilizing angle reading...");
  for (int i = 0; i < 100; i++) {
    readAndProcessSensor();
    delay(10);
    lastLoopTime = millis(); // Update timing during stabilization
  }
  
  // Set target to stabilized angle
  setpoint = pitch;
  Serial.println("Stabilized pitch: " + String(pitch, 2) + "°");
  Serial.println("Target automatically set to: " + String(setpoint, 2) + "°");
}

void quickCalibrate() {
  Serial.println("\n=== Quick Balance Point Calibration ===");
  Serial.println("Hold robot in desired balance position...");
  Serial.println("Quick calibration in 2 seconds...");
  
  for (int i = 2; i > 0; i--) {
    Serial.println(String(i) + "...");
    delay(1000);
  }
  
  // Take several readings to average out noise
  float angleSum = 0;
  int samples = 50;
  
  Serial.println("Sampling current angle...");
  for (int i = 0; i < samples; i++) {
    readAndProcessSensor();
    angleSum += (pitch - pitchOffset); // Use raw pitch without offset
    delay(20);
  }
  
  float currentRawAngle = angleSum / samples;
  pitchOffset = -currentRawAngle; // Set offset to make current position = 0
  
  Serial.println("Quick calibration complete!");
  Serial.println("Balance point set to current position");
  Serial.println("Pitch offset: " + String(pitchOffset, 2) + "°");
  
  // Verify
  delay(200);
  readAndProcessSensor();
  Serial.println("New corrected pitch: " + String(pitch, 2) + "°");
}

void showCurrentAngle() {
  readAndProcessSensor();
  Serial.println("\n=== Current Angle Readings ===");
  Serial.println("Raw Pitch (from sensor): " + String(pitch - pitchOffset, 2) + "°");
  Serial.println("Pitch Offset: " + String(pitchOffset, 2) + "°");
  Serial.println("Corrected Pitch: " + String(pitch, 2) + "°");
  Serial.println("Target Angle: " + String(setpoint, 2) + "°");
  Serial.println("Error: " + String(setpoint - pitch, 2) + "°");
}

void showPitchOffset() {
  Serial.println("\n=== Pitch Offset Info ===");
  Serial.println("Current pitch offset: " + String(pitchOffset, 2) + "°");
  Serial.println("This value is added to sensor reading to correct balance point");
  Serial.println("Use 'offset2.5' to set offset or 'quickcal' to auto-set");
}

// Reverse motor direction
void reverseMotorDirection() {
  motorReversed = !motorReversed;
  Serial.println("Motor direction: " + String(motorReversed ? "REVERSED" : "NORMAL"));
  Serial.println("Use this if robot falls in wrong direction during balancing");
}

// Toggle directional PID
void toggleDirectionalPID() {
  useDirectionalPID = !useDirectionalPID;
  Serial.println("Directional PID: " + String(useDirectionalPID ? "ENABLED" : "DISABLED"));
  
  if (useDirectionalPID) {
    Serial.println("Now using separate gains for each tilt direction:");
    Serial.println("Forward tilt: Kp=" + String(KpForward) + " Kd=" + String(KdForward));
    Serial.println("Backward tilt: Kp=" + String(KpBackward) + " Kd=" + String(KdBackward));
    Serial.println("Use 'pf25' to set forward Kp, 'pb15' for backward Kp");
  } else {
    Serial.println("Using standard PID: Kp=" + String(Kp) + " Ki=" + String(Ki) + " Kd=" + String(Kd));
  }
}

// Safety monitoring function
bool performSafetyChecks() {
  unsigned long currentTime = millis();
  
  // Only check loop timing if enabled (after initialization)
  if (enableTimingCheck && lastLoopTime != 0) {
    unsigned long timeSinceLoop = currentTime - lastLoopTime;
    
    // Check for loop timing issues (between 50ms-500ms indicates real problem)
    if (timeSinceLoop > 50 && timeSinceLoop < 500) {
      Serial.println("WARNING: Loop timing issue detected!");
      Serial.println("Time since last loop: " + String(timeSinceLoop) + "ms");
    }
  }
  
  // Check for sensor reading issues
  if (isnan(pitch) || isinf(pitch) || abs(pitch) > 180) {
    Serial.println("EMERGENCY: Invalid sensor reading!");
    Serial.println("Pitch value: " + String(pitch));
    emergencyStop = true;
    return false;
  }
  
  // Check PID output for reasonable values
  if (isnan(output) || isinf(output)) {
    Serial.println("EMERGENCY: Invalid PID output!");
    Serial.println("Output value: " + String(output));
    emergencyStop = true;
    return false;
  }
  
  return true;
}

// Clear emergency stop condition
void clearEmergencyStop() {
  emergencyStop = false;
  motorRunTime = 0;
  lastMotorStart = 0;
  stopMotors();
  Serial.println("Emergency stop cleared. System reset.");
  Serial.println("Motors are now stopped. Use 'balance' or 'start' to resume.");
}