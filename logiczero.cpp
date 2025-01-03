//with servo
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

MPU6050 mpu;

Servo servoX;  //  Servo object for X-axis
Servo servoY;  //  Servo object for Y-axis

const int SDA_PIN = 5;  // GPIO5 - SDA
const int SCL_PIN = 4;  // GPIO4 - SCL
const int servopin1 = 12; //servo x axis
const int servopin2 = 14; //servo y axis 

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

}

void loop() {
  float ax,ay,az;
  float accx,accy,accz;

  float gx,gy,gz;
  float gyrox,gyroy,gyroz;

  ax = mpu.getAccelerationX();
  ay = mpu.getAccelerationY();
  az = mpu.getAccelerationZ();

  accx = ax / 16384.0;
  accy = ay / 16384.0;
  accz = az / 16384.0;
  
  gx = mpu.getRotationX();
  gy = mpu.getRotationY();
  gz = mpu.getRotationZ();

  gyrox = gx / 131.0;
  gyroy = gy / 131.0;
  gyroz = gz / 131.0;

//accln values
  Serial.print("Acceleration (g): X = ");
  Serial.print(ax / 16384.0);  
  Serial.print(", Y = ");
  Serial.print(ay / 16384.0);
  Serial.print(", Z = ");
  Serial.println(az / 16384.0);

//gyro values
  Serial.print("Gyroscope (°/s): X = ");
  Serial.print(gx / 131.0); 
  Serial.print(", Y = ");
  Serial.print(gy / 131.0);
  Serial.print(", Z = ");
  Serial.println(gz / 131.0);

  int servoAngleX = map(ax, -17000, 17000, 0, 180);
  int servoAngleY = map(ay, -17000, 17000, 0, 180);

  if(gyrox == 0)
  {servoX.write(0);}
  if(0.5>=gyrox&& gyrox>0)
  {servoX.write(15);}
  if(1>=gyrox&&gyrox>0.5)
  {servoX.write(30);}
  if(gyrox>1)
  {servoX.write(45);}

  if(gyroy == 0)
  {servoY.write(0);}
  if(0.5>=gyroy && gyroy>0)
  {servoY.write(15);}
  if(1>=gyroy && gyroy>=0.5)
  {servoY.write(30);}
  if(gyroy>1)
  {servoY.write(45);}


  delay(500);



}
