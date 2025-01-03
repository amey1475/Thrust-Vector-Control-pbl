#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

MPU6050 mpu;

Servo servoX;  // Create Servo object for X-axis
Servo servoY;  // Create Servo object for Y-axis

const int SDA_PIN = 5;  // GPIO5 - SDA
const int SCL_PIN = 4;  // GPIO4 - SCL
const int servopin1 = 12; // Servo X-axis
const int servopin2 = 14; // Servo Y-axis 

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  
  Serial.println("MPU6050 initialized successfully.");

  servoX.attach(servopin1);
  servoY.attach(servopin2);
  Serial.println("Servos attached successfully.");
}

void loop() {
  float ax = mpu.getAccelerationX();
  float ay = mpu.getAccelerationY();
  float gx = mpu.getRotationX();
  float gy = mpu.getRotationY();

  // Normalize acceleration values
  float accx = ax / 16384.0;
  float accy = ay / 16384.0;

  // Normalize gyroscope values
  float gyrox = gx / 131.0;
  float gyroy = gy / 131.0;

  // Print debug data
  Serial.print("Acceleration (g): X = ");
  Serial.print(accx);
  Serial.print(", Y = ");
  Serial.println(accy);

  Serial.print("Gyroscope (°/s): X = ");
  Serial.print(gyrox);
  Serial.print(", Y = ");
  Serial.println(gyroy);

  // Map accelerometer data to servo angles
  int servoAngleX = map(ax, -17000, 17000, 0, 180);
  int servoAngleY = map(ay, -17000, 17000, 0, 180);

  // Control servos based on gyroscope data
  if (gyrox == 0) servoX.write(0);
  else if (gyrox > 0 && gyrox <= 0.5) servoX.write(15);
  else if (gyrox > 0.5 && gyrox <= 1) servoX.write(30);
  else if (gyrox > 1) servoX.write(45);

  if (gyroy == 0) servoY.write(0);
  else if (gyroy > 0 && gyroy <= 0.5) servoY.write(15);
  else if (gyroy > 0.5 && gyroy <= 1) servoY.write(30);
  else if (gyroy > 1) servoY.write(45);

  delay(500);
}
