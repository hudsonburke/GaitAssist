#include <Arduino.h>

#define LOAD_CELL_PIN A0
#define MOTOR_PIN 3

#define SLACK_THRESHOLD 100 // Threshold for slack detection

#define MOTOR_VOLT_MIN 1.5 // V
#define MOTOR_VOLT_MAX 4.0 // V
#define MOTOR_RPM_MAX 4000 // rpm
#define MOTOR_ANG_VEL_MAX (MOTOR_RPM_MAX * PI / 30.0) // rad/s

#define KU 7 // Ultimate gain
#define TU 0.5 // Ultimate period (seconds)
#define KP KU //(0.6 * KU) // Proportional gain
#define KI (0.1 * KU / TU) // Integral gain
#define KD (0.075 * KU * TU) // Derivative gain
#define WINDOW_SIZE 10 // Number of samples for integral calculation

#define TARGET_TENSION 25 // Newtons
#define M_DISK 0.5 // kg
#define R_DISK 0.15 // m
#define I_DISK (0.5 * M_DISK * R_DISK * R_DISK) // kg*m^2
#define GEAR_RATIO 32 // 32:1

// Load cell and amplifier parameters
#define LOAD_CAPACITY 900 // N
#define AMP_VOLT_MIN 0.0 // V
#define AMP_VOLT_MAX 5.0 // V
#define AMP_VOLT_MID (AMP_VOLT_MIN + AMP_VOLT_MAX) / 2.0 // V

#define ADC_RESOLUTION 1024 // 10-bit ADC resolution
#define ADC_REF_VOLTAGE 5.0 // Reference voltage for ADC

// TODO: On the first loop, the initial angular velocity is set to maximum because there is slack 
// in the system and the motor needs to accelerate quickly to reach the target tension
#define START_RPM 0.0 // MOTOR_RPM_MAX
#define START_ANG_VEL (START_RPM * PI / 30.0) // rad/s
#define TRIAL_DURATION 10000 // milliseconds

// put function declarations here:

void setup() {
  // put your setup code here, to run once:
  pinMode(MOTOR_PIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Setup complete");
  delay(1000);
  Serial.println("Enter '1' to start the motor");
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '1') {
        Serial.println("Starting motor");
        break;
      }
    }
  }
  Serial.println("Motor started");
  analogWrite(MOTOR_PIN, START_ANG_VEL); // Start with motor off
}

void loop() {
  
  static float prevError = 0.0;
  static float integral = 0.0;
  static float errorWindow[WINDOW_SIZE] = {0};
  static int windowIndex = 0;
  static float prevAngVel = START_ANG_VEL;

  
  int loadCellValue = analogRead(LOAD_CELL_PIN);
  Serial.print(">loadCellValue:");
  Serial.println(loadCellValue);

  float loadCellVoltage = loadCellValue * (ADC_REF_VOLTAGE / (ADC_RESOLUTION - 1)); // Convert to voltage
  Serial.print(">loadCellVoltage:");
  Serial.println(loadCellVoltage);

  // Convert load cell voltage to force: The middle of the range is 0N, negative force moves voltage down
  // and positive force moves voltage up. The slope is (LOAD_CAPACITY / (AMP_VOLT_MAX - AMP_VOLT_MIN))
  float loadCellForce = (loadCellVoltage - AMP_VOLT_MID) * (LOAD_CAPACITY / (AMP_VOLT_MAX - AMP_VOLT_MIN));
  Serial.print(">loadCellForce:");
  Serial.println(loadCellForce);
  
  float tension = loadCellForce / 2.0;
  Serial.print(">tension:");
  Serial.println(tension);

  // PID control
  float forceError = TARGET_TENSION - tension;
  Serial.print(">forceError:");
  Serial.println(forceError);
  integral -= errorWindow[0];
  integral += forceError;
  Serial.print(">integral:");
  Serial.println(integral);
  errorWindow[windowIndex] = forceError;
  windowIndex = (windowIndex + 1) % WINDOW_SIZE;
  float derivative = forceError - prevError;
  Serial.print(">derivative:");
  Serial.println(derivative);
  prevError = forceError;
  float controlSignal = KP * forceError + KI * integral + KD * derivative;
  Serial.print(">controlSignal:");
  Serial.println(controlSignal);
  
  // Control motor speed
  float targetAngVel = prevAngVel + (R_DISK * controlSignal / I_DISK) * GEAR_RATIO;
  Serial.print(">targetAngVel:");
  Serial.println(targetAngVel);
  prevAngVel = targetAngVel;

  // Map angular velocity to motor voltage
  // Assuming the motor voltage is between MOTOR_VOLT_MIN and MOTOR_VOLT_MAX
  // and the angular velocity is between 0 and MOTOR_ANG_VEL_MAX
  float motorVoltage = map(targetAngVel, 0, MOTOR_ANG_VEL_MAX, MOTOR_VOLT_MIN, MOTOR_VOLT_MAX);
  Serial.print(">motorVoltage:");
  Serial.println(motorVoltage);

  float pwmAngVel = map(motorVoltage, MOTOR_VOLT_MIN, MOTOR_VOLT_MAX, 0, 255);
  pwmAngVel = constrain(pwmAngVel, 0, 255); // Constrain to valid PWM range
  Serial.print(">pwmAngVel:");
  Serial.println(pwmAngVel);
  
  analogWrite(MOTOR_PIN, pwmAngVel);
  static unsigned long startTime = millis();
  if (millis() - startTime > TRIAL_DURATION) {
    Serial.println("Stopping motor");
    analogWrite(MOTOR_PIN, 0); // Stop the motor
    Serial.println("Enter '1' to start the motor again or '2' to exit");
    while (true) {
      if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
          case '1':
            Serial.println("Starting motor");
            for (int i = 0; i < WINDOW_SIZE; i++) {
              errorWindow[i] = 0.0; // Reset the error window
            }
            prevError = 0.0; // Reset the previous error
            integral = 0.0; // Reset the integral
            prevAngVel = START_ANG_VEL; // Reset the previous angular velocity
            startTime = millis();
            break;
          case '2':
            Serial.println("Exiting");
            return; // Exit the program
          // case '3':
          //   // Reverse motor direction
          //   Serial.println("Reversing motor direction");

          default:
            break;
        }
      } 
    }
  }
}

// put function definitions here: