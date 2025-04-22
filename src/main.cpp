#include <Arduino.h>

// Motor parameters
#define MOTOR_PIN 3
#define PI 3.14159265358979323846 // Define PI explicitly
#define MOTOR_VOLT_MIN 1.5 // V
#define MOTOR_VOLT_MAX 4.0 // V
#define MOTOR_RPM_MAX 4000 // rpm
#define MOTOR_ANG_VEL_MAX (MOTOR_RPM_MAX * PI / 30.0) // rad/s
#define MOTOR_EFFICIENCY 0.85 // Motor efficiency factor
#define MOTOR_VOLT_PER_ANG_VEL ((MOTOR_VOLT_MAX - MOTOR_VOLT_MIN) / MOTOR_ANG_VEL_MAX)

// PID parameters
#define KU 7 // Ultimate gain
#define TU 0.33 // Ultimate period (seconds)
#define KP (0.6 * KU) // Proportional gain
#define KI (1.2 * KU / TU) // Integral gain
#define KD (1.0 * KU * TU) // Derivative gain
#define WINDOW_SIZE 10 // Number of samples for integral calculation

#define M_DISK 0.5 // kg
#define R_DISK 0.15 // m
#define I_DISK (0.5 * M_DISK * R_DISK * R_DISK) // kg*m^2
#define GEAR_RATIO 32 // 32:1

// Load cell and amplifier parameters
#define LOAD_CELL_PIN A0
#define LOAD_CAPACITY 900 // N (in single direction i.e. AMP_VOLT_MID to AMP_VOLT_MAX)
#define AMP_VOLT_MIN -10.0 // V
#define AMP_VOLT_MAX 10.0 // V
#define AMP_VOLT_MID (AMP_VOLT_MIN + AMP_VOLT_MAX) / 2.0 // V

#define LOAD_PER_VOLT (LOAD_CAPACITY / (AMP_VOLT_MAX - AMP_VOLT_MID))

// ADC and PWM parameters
#define ADC_RESOLUTION 1024 // 10-bit ADC resolution
#define ADC_REF_VOLTAGE 5.0 // Reference voltage for ADC
#define ADC_VOLT_PER_UNIT (ADC_REF_VOLTAGE / (ADC_RESOLUTION - 1))

#define PWM_REF_VOLTAGE 5.0 // Reference voltage for PWM
#define PWM_MIN_VOLT 0.0 // Minimum voltage for PWM
#define PWM_MAX_VOLT 5.0 // Maximum voltage for PWM
#define PWM_RESOLUTION 255 // 8-bit PWM resolution
#define PWM_VOLT_PER_UNIT (PWM_REF_VOLTAGE / PWM_RESOLUTION) // V/step

#define START_RPM 0.0
#define START_ANG_VEL (START_RPM * PI / 30.0) // rad/s
#define TRIAL_DURATION 10000 // milliseconds

#define LOOP_FREQUENCY 75 // Hz (debug prints can significantly reduce the loop frequency due to serial communication delays)
#define LOOP_DELAY (1000 / LOOP_FREQUENCY) // milliseconds

#define DEBUG true // Set to false to disable debug prints

float targetTension = 0.0;

void setup() {
  // put your setup code here, to run once:
  pinMode(MOTOR_PIN, OUTPUT);
  Serial.begin(115200);
  if (DEBUG) {
    Serial.println("Warning: Debug mode is enabled. This may affect performance.");
  }
  Serial.println("Setup complete");
  Serial.println("Enter the target tension (N):");
  while (true) {
    if (Serial.available()) {
      targetTension = Serial.parseFloat();
      if (targetTension > 0.0) {
        Serial.print("Target tension set to: ");
        Serial.println(targetTension);
        break;
      } else {
        Serial.println("Invalid input. Please enter a positive number.");
      }
    }
  }
  Serial.println("Enter '1' to start the motor:");
  // Wait for user input to start the motor
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '1') {
        Serial.println("Starting motor");
        break;
      }
    }
  }
  float startMotorVoltage = MOTOR_VOLT_MIN + START_ANG_VEL * MOTOR_VOLT_PER_ANG_VEL;
  int startPwmValue = int(startMotorVoltage / PWM_VOLT_PER_UNIT);
  startPwmValue = constrain(startPwmValue, 0, 255);
  analogWrite(MOTOR_PIN, startPwmValue); // Start with motor off
  analogWrite(MOTOR_PIN, START_ANG_VEL); // Start with motor off
}

void loop() {
  static unsigned long lastLoopTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastLoopTime < LOOP_DELAY) {
    return; // Skip this iteration to maintain the desired frequency
  }

  float deltaTime = (currentTime - lastLoopTime) / 1000.0; // Convert to seconds
  lastLoopTime = currentTime;

  static float prevError = 0.0;
  static float integral = 0.0;
  static float errorWindow[WINDOW_SIZE] = {0};
  static int windowIndex = 0;
  static float prevAngVel = START_ANG_VEL;

  // Read load cell value
  int loadCellValue = analogRead(LOAD_CELL_PIN);

  // Convert ADC value to voltage
  float loadCellVoltage = loadCellValue * ADC_VOLT_PER_UNIT;

  // Convert load cell voltage to force
  float loadCellForce = (loadCellVoltage - AMP_VOLT_MID) * LOAD_PER_VOLT;
  float tension = loadCellForce / 2.0;

  // PID calculations
  float forceError = targetTension - tension;
  integral += (forceError * deltaTime) - errorWindow[windowIndex];
  errorWindow[windowIndex] = forceError;
  windowIndex = (windowIndex + 1) % WINDOW_SIZE;

  float derivative = (deltaTime > 0.0) ? (forceError - prevError) / deltaTime : 0.0;

  float controlSignal = KP * forceError + KI * integral + KD * derivative;

  // Calculate motor torque and angular velocity
  float torque = R_DISK * controlSignal; // Torque applied to the motor
  float adjustedTorque = torque * MOTOR_EFFICIENCY; // Adjusted for motor efficiency
  float targetAngVel = prevAngVel + ((adjustedTorque / I_DISK) * GEAR_RATIO * deltaTime);
  targetAngVel = constrain(targetAngVel, 0.0, MOTOR_ANG_VEL_MAX);
  prevAngVel = targetAngVel;

  // Map angular velocity to motor voltage
  float motorVoltage = MOTOR_VOLT_MIN + targetAngVel * MOTOR_VOLT_PER_ANG_VEL;

  // Convert motor voltage to PWM value
  int pwmValue = int(motorVoltage / PWM_VOLT_PER_UNIT);
  pwmValue = constrain(pwmValue, 0, 255);

  analogWrite(MOTOR_PIN, pwmValue);

  // Debug prints
  if (DEBUG) {
    float loopFrequency = 1.0 / deltaTime; // Hz
    Serial.print(">loopFrequency:"); Serial.println(loopFrequency);
    Serial.print(">tension:"); Serial.println(tension);
    Serial.print(">forceError:"); Serial.println(forceError);
    Serial.print(">integral:"); Serial.println(integral);
    Serial.print(">derivative:"); Serial.println(derivative);
    Serial.print(">controlSignal:"); Serial.println(controlSignal);
  }

  // Stop motor after trial duration
  static unsigned long startTime = millis();
  if (millis() - startTime > TRIAL_DURATION) {
    analogWrite(MOTOR_PIN, 0); // Stop the motor
    Serial.println("Trial duration exceeded. Stopping motor.");
    while (true) {
      // Wait indefinitely
    }
  }
}
