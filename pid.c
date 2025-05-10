#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

MPU6050 mpu;
Servo servoX, servoY;

const int SDA_PIN = 5;
const int SCL_PIN = 4;
const int servopin1 = 12;
const int servopin2 = 14;

// Desired angle setpoints (target angles)
float targetAngleX = 0;  // Desired roll angle
float targetAngleY = 0;  // Desired pitch angle

// PID constants (these may need tuning)
float kp = 15.0;  // Proportional gain
float ki = 5.0;   // Integral gain
float kd = 1.0;   // Derivative gain

// PID variables for X and Y axes
float previousErrorX = 0, previousErrorY = 0;
float integralX = 0, integralY = 0;

// Servo angle limits and idle position
const int servoMinAngle = 60;
const int servoMaxAngle = 120;
const int servoIdleAngle = 90;

void setup() {
  Serial.begin(9600);  // Match baud rate with Python code
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  Serial.println("MPU6050 initialized successfully.");

  // Attach servos to specified pins
  servoX.attach(servopin1);
  servoY.attach(servopin2);

  // Set servos to idle position initially
  servoX.write(servoIdleAngle);
  servoY.write(servoIdleAngle);
}

void loop() {
  // Read gyro and accel values for pitch and roll
  float gx = mpu.getRotationX();
  float gy = mpu.getRotationY();
  float gz = mpu.getRotationZ();
  float ax = mpu.getAccelerationX();
  float ay = mpu.getAccelerationY();
  float az = mpu.getAccelerationZ();

  // Convert raw gyro values to degrees per second
  float gyrox = gx / 131.0;
  float gyroy = gy / 131.0;
  float gyroz = gz / 131.0;

  // PID control for X-axis (Roll control)
  float errorX = targetAngleX - gyrox;
  integralX += errorX;
  float derivativeX = errorX - previousErrorX;
  int servoAngleX = servoIdleAngle + (kp * errorX + ki * integralX + kd * derivativeX);
  previousErrorX = errorX;

  // PID control for Y-axis (Pitch control)
  float errorY = targetAngleY - gyroy;
  integralY += errorY;
  float derivativeY = errorY - previousErrorY;
  int servoAngleY = servoIdleAngle + (kp * errorY + ki * integralY + kd * derivativeY);
  previousErrorY = errorY;

  // Constrain the servo angles to stay within limits
  servoAngleX = constrain(servoAngleX, servoMinAngle, servoMaxAngle);
  servoAngleY = constrain(servoAngleY, servoMinAngle, servoMaxAngle);

  // Move the servos to the calculated angles
  servoX.write(servoAngleX);
  servoY.write(servoAngleY);

  // Transmit data to match Python dashboard format
  Serial.print(gyrox); Serial.print(",");
  Serial.print(gyroy); Serial.print(",");
  Serial.print(gyroz); Serial.print(",");
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print("0"); Serial.print(",");  // Placeholder for center of mass x
  Serial.println("0");                   // Placeholder for center of mass y

  delay(100);  // Match with animation update interval
}
