/*
  Self-Balancing Robot (single-loop, tilt-only)
  Hardware: ESP32, MPU6050 (I2Cdevlib DMP), L298N, DC12-USB5V Buck Converter, 2x 3-6V DC gear motors, 2x 3.7V 18650 Batteries, 9V Battery

  Libraries Used (Library Manager or GitHub):
    - I2Cdevlib-MPU6050  (https://github.com/jrowberg/i2cdevlib)
    - I2Cdevlib-Core
    - PID_v1 by Brett Beauregard

  Two Calibrations Used
    1. IMU offset calibration (sensor bias) via RUN_CALIBRATION = true
    
    2. Mechanical balance point (BALANCE_SETPOINT) via pitch in serial monitor
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <PID_v1_bc.h>

// Pin Configurations
#define SDA_PIN           21
#define SCL_PIN           22
#define MPU_INTERRUPT_PIN 15

#define IN1 33
#define IN2 25
#define IN3 26
#define IN4 27
#define ENA 32  // PWM enable, motor A (left)
#define ENB 14   // PWM enable, motor B (right)

// Calibrations
#define RUN_CALIBRATION false
// set true once, run, copy offsets below, then set false

// Robot
#define XA_OFFSET -328
#define YA_OFFSET 9
#define ZA_OFFSET 826
#define XG_OFFSET 14
#define YG_OFFSET 70
#define ZG_OFFSET -13

// Pitch angle (degrees) reported by the DMP when the robot is physically upright.
double BALANCE_SETPOINT = 0;

// Robot is considered fallen past this angle from setpoint; motors cut for safety.
#define FALL_LIMIT_DEG 35

// PID Tuning
double Kp = 30;
double Ki = 0.5;
double Kd = 0.75;

// Global Variables
MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

double pitchInput, pidOutput;
PID balancePID(&pitchInput, &pidOutput, &BALANCE_SETPOINT, Kp, Ki, Kd, DIRECT);

// Motor Control
void driveMotors(int pwmValue) {
  pwmValue = constrain(pwmValue, -200, 200);

  if (pwmValue >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  int pwmMagnitude = abs(pwmValue);
  analogWrite(ENA, pwmMagnitude);
  analogWrite(ENB, pwmMagnitude);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Setup
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stopMotors();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  pinMode(MPU_INTERRUPT_PIN, INPUT);

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed. Check wiring/address.");
    while (1) {}
  }

  devStatus = mpu.dmpInitialize();

  if (RUN_CALIBRATION) {
    Serial.println("Running IMU calibration. Keep the robot flat and still.");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println("Calibration done. Copy these offsets into the #define block:");
    mpu.PrintActiveOffsets();
    Serial.println("Then set RUN_CALIBRATION to false and re-upload.");
    while (1) {} // halt, no motor operation during calibration mode
  } else {
    mpu.setXAccelOffset(XA_OFFSET);
    mpu.setYAccelOffset(YA_OFFSET);
    mpu.setZAccelOffset(ZA_OFFSET);
    mpu.setXGyroOffset(XG_OFFSET);
    mpu.setYGyroOffset(YG_OFFSET);
    mpu.setZGyroOffset(ZG_OFFSET);
  }

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print("DMP initialization failed, code: ");
    Serial.println(devStatus);
    while (1) {}
  }

  balancePID.SetMode(AUTOMATIC);
  balancePID.SetOutputLimits(-200, 200);
  balancePID.SetSampleTime(10);
}

// Main Loop
void loop() {
  if (!dmpReady) return;

  if (!mpuInterrupt && fifoCount < packetSize) return;

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
    return;
  }

  if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    pitchInput = ypr[1] * 180.0 / M_PI;

    if (abs(pitchInput - BALANCE_SETPOINT) > FALL_LIMIT_DEG) {
      stopMotors();
      balancePID.SetMode(MANUAL);
      pidOutput = 0;
    } else {
      balancePID.SetMode(AUTOMATIC);
      balancePID.Compute();
      driveMotors((int)pidOutput);
    }

  // Debug output
  // Serial.print("pitch: ");
  // Serial.print(pitchInput);
  // Serial.print("  output: ");
  // Serial.println(pidOutput);
  }
}
