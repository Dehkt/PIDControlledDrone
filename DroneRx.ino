#include <Wire.h>
#include <ESP32Servo.h>
#include <nRF24L01.h>
#include <RF24.h>

// Motors
Servo motor1, motor2, motor3, motor4;
const int motor1Pin = 16;
const int motor2Pin = 17;
const int motor3Pin = 18; 
const int motor4Pin = 19;

// MPU6050 IMU
const int MPU6050_ADDR = 0x68;
int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
float Ax, Ay, Az;
float Gx, Gy, Gz;

// PID Control
float Kp = 1.0; 
float Ki = 0.0; 
float Kd = 0.0;  

float pitchSetpoint = 0.0, rollSetpoint = 0.0, yawSetpoint = 0.0;
float pitchError, rollError, yawError;
float pitchPrevError = 0.0, rollPrevError = 0.0, yawPrevError = 0.0;
float pitchIntegral = 0.0, rollIntegral = 0.0, yawIntegral = 0.0;
float motorSpeed1, motorSpeed2, motorSpeed3, motorSpeed4;

// nRF24L01 settings
const uint64_t pipeIn = 0xE8E8F0F0E1LL; // Must match the transmitter
RF24 radio(7, 8); // CE and CSN pins

struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
};

MyData receivedData;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  MPU6050_Init();

  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);

  delay(1500);
  // Calibration Cycle
  for(int i = 0; i < 5; i++){
    motor1.writeMicroseconds(2000);
    motor2.writeMicroseconds(2000);
    motor3.writeMicroseconds(2000);
    motor4.writeMicroseconds(2000);
    delay(2000);
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
  }

}

void MPU6050_Init() {
  Wire.beginTransmission(MPU6050_ADDR); 
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Set to zero to wake up the MPU6050
  if (Wire.endTransmission(true) != 0) {
    Serial.println("Failed to initialize MPU6050");
  }
  
  delay(100); 

  // Set accelerometer configuration 
  Wire.beginTransmission(MPU6050_ADDR);  
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // Set to ±2g sensitivity 

  Wire.beginTransmission(MPU6050_ADDR);  
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // Set to ±250 degrees/sec

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x19);  // SMPLRT_DIV register
  Wire.write(0x07);  // Set sample rate to 1kHz / (1 + 7) = 125Hz

}

void MPU6050_Read_Accel() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // Starting with ACCEL_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);  // Read 6 bytes of data

  Accel_X_RAW = (Wire.read() << 8 | Wire.read());
  Accel_Y_RAW = (Wire.read() << 8 | Wire.read());
  Accel_Z_RAW = (Wire.read() << 8 | Wire.read());

  Ax = Accel_X_RAW / 16384.0;
  Ay = Accel_Y_RAW / 16384.0;
  Az = Accel_Z_RAW / 16384.0;
}

void MPU6050_Read_Gyro() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);  // Starting with GYRO_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);  // Read 6 bytes of data

  Gyro_X_RAW = (Wire.read() << 8 | Wire.read());
  Gyro_Y_RAW = (Wire.read() << 8 | Wire.read());
  Gyro_Z_RAW = (Wire.read() << 8 | Wire.read());

  Gx = Gyro_X_RAW / 131.0;
  Gy = Gyro_Y_RAW / 131.0;
  Gz = Gyro_Z_RAW / 131.0;
}

void loop() {
  if (radio.available()) {
    radio.read(&receivedData, sizeof(MyData));
  }

  // Update setpoints based on user inputs
  throttleControl(receivedData.throttle);
  pitchSetpoint = map(receivedData.pitch, 0, 255, -45, 45);
  rollSetpoint = map(receivedData.roll, 0, 255, -45, 45);
  yawSetpoint = map(receivedData.yaw, 0, 255, -90, 90);

  // Read IMU data
  MPU6050_Read_Gyro();

  // Compute PID for pitch, roll, and yaw
  computePID();

  // Adjust motor speeds based on PID and user throttle input
  applyMotorSpeeds();
  delay(20);  // Loop timing

  delay(20);  // Short delay for loop stability
}

   
void computePID() {
  // Calculate errors for PID
  pitchError = pitchSetpoint - Gx;
  rollError = rollSetpoint - Gy;
  yawError = yawSetpoint - Gz;

  // PID for pitch
  float pitchP = Kp * pitchError;
  pitchIntegral += pitchError;
  float pitchI = Ki * pitchIntegral;
  float pitchD = Kd * (pitchError - pitchPrevError);
  float pitchOutput = pitchP + pitchI + pitchD;
  pitchPrevError = pitchError;

  // PID for roll
  float rollP = Kp * rollError;
  rollIntegral += rollError;
  float rollI = Ki * rollIntegral;
  float rollD = Kd * (rollError - rollPrevError);
  float rollOutput = rollP + rollI + rollD;
  rollPrevError = rollError;

  // PID for yaw
  float yawP = Kp * yawError;
  yawIntegral += yawError;
  float yawI = Ki * yawIntegral;
  float yawD = Kd * (yawError - yawPrevError);
  float yawOutput = yawP + yawI + yawD;
  yawPrevError = yawError;

  // Motor speed adjustments based on PID
  motorSpeed1 = 1500 + pitchOutput + rollOutput - yawOutput;
  motorSpeed2 = 1500 + pitchOutput - rollOutput + yawOutput;
  motorSpeed3 = 1500 - pitchOutput + rollOutput + yawOutput;
  motorSpeed4 = 1500 - pitchOutput - rollOutput - yawOutput;
}

void throttleControl(byte throttle) {
  int baseSpeed = map(throttle, 0, 255, 1000, 2000);
  motorSpeed1 = baseSpeed;
  motorSpeed2 = baseSpeed;
  motorSpeed3 = baseSpeed;
  motorSpeed4 = baseSpeed;
}

void applyMotorSpeeds() {
  motorSpeed1 = constrain(motorSpeed1, 1000, 2000);
  motorSpeed2 = constrain(motorSpeed2, 1000, 2000);
  motorSpeed3 = constrain(motorSpeed3, 1000, 2000);
  motorSpeed4 = constrain(motorSpeed4, 1000, 2000);

  motor1.writeMicroseconds(motorSpeed1);
  motor2.writeMicroseconds(motorSpeed2);
  motor3.writeMicroseconds(motorSpeed3);
  motor4.writeMicroseconds(motorSpeed4);
}
 