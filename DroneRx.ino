#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP32Servo.h>

// Motors
Servo motor1, motor2, motor3, motor4;
const int motor1Pin = 16;
const int motor2Pin = 17;
const int motor3Pin = 18;
const int motor4Pin = 19;

// MPU6050 IMU
const int MPU6050_ADDR = 0x68; // MPU6050 I2C address
int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
float Ax, Ay, Az;
float Gx, Gy, Gz;

// RF24 (Receiver)
RF24 radio(7, 8);  // CE and CSN pins
const uint64_t pipeIn = 0xE8E8F0F0E8E8;

// The data struct received from the transmitter
struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
};

MyData receivedData;

// PID Controller Variables
float Kp = 1.0, Ki = 0.0, Kd = 0.0;
float pitchSetpoint = 0.0, rollSetpoint = 0.0, yawSetpoint = 0.0;
float pitchError, rollError, yawError;
float pitchPrevError = 0.0, rollPrevError = 0.0, yawPrevError = 0.0;
float pitchIntegral = 0.0, rollIntegral = 0.0, yawIntegral = 0.0;
float motorSpeed1, motorSpeed2, motorSpeed3, motorSpeed4;

// Setup Function
void setup() {
  Serial.begin(9600);
  Wire.begin();
  MPU6050_Init();

  // Setup Motors
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);

  // Calibration
  motorCalibration();

  // Setup RF24
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, pipeIn);
  radio.startListening();  // Set as receiver
}

// Motor Calibration Function
void motorCalibration() {
  for(int i = 0; i < 5; i++) {
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

// MPU6050 Initialization
void MPU6050_Init() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // Power management register
  Wire.write(0x00);  // Wake up the MPU6050
  Wire.endTransmission(true);
  
  delay(100);  // Small delay for stability
  
  // Accelerometer configuration
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // Set to ±2g
  Wire.endTransmission(true);

  // Gyroscope configuration
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // Set to ±250 degrees/sec
  Wire.endTransmission(true);
}

// Read Accelerometer
void MPU6050_Read_Accel() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // Starting with ACCEL_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  Accel_X_RAW = (Wire.read() << 8 | Wire.read());
  Accel_Y_RAW = (Wire.read() << 8 | Wire.read());
  Accel_Z_RAW = (Wire.read() << 8 | Wire.read());

  Ax = Accel_X_RAW / 16384.0;
  Ay = Accel_Y_RAW / 16384.0;
  Az = Accel_Z_RAW / 16384.0;
}

// Read Gyroscope
void MPU6050_Read_Gyro() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);  // Starting with GYRO_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  Gyro_X_RAW = (Wire.read() << 8 | Wire.read());
  Gyro_Y_RAW = (Wire.read() << 8 | Wire.read());
  Gyro_Z_RAW = (Wire.read() << 8 | Wire.read());

  Gx = Gyro_X_RAW / 131.0;
  Gy = Gyro_Y_RAW / 131.0;
  Gz = Gyro_Z_RAW / 131.0;
}

// Main Loop
void loop() {
  // Read IMU data
  MPU6050_Read_Accel();
  MPU6050_Read_Gyro();

  // Check if data is available from transmitter
  if (radio.available()) {
    radio.read(&receivedData, sizeof(MyData));
    // Map received joystick values to setpoints
    pitchSetpoint = map(receivedData.pitch, 0, 255, -45, 45);  // Adjust range as needed
    rollSetpoint = map(receivedData.roll, 0, 255, -45, 45);
    yawSetpoint = map(receivedData.yaw, 0, 255, -180, 180);
  }

  // PID Control calculations
  pidControl();
  
  // Update motor speeds
  updateMotors();
}

// PID Control Function
void pidControl() {
  // Calculate errors
  pitchError = pitchSetpoint - Gx;
  rollError = rollSetpoint - Gy;
  yawError = yawSetpoint - Gz;

  // Proportional term
  float pitchP = Kp * pitchError;
  float rollP = Kp * rollError;
  float yawP = Kp * yawError;

  // Integral term
  pitchIntegral += pitchError;
  rollIntegral += rollError;
  yawIntegral += yawError;

  float pitchI = Ki * pitchIntegral;
  float rollI = Ki * rollIntegral;
  float yawI = Ki * yawIntegral;

  // Derivative term
  float pitchD = Kd * (pitchError - pitchPrevError);
  float rollD = Kd * (rollError - rollPrevError);
  float yawD = Kd * (yawError - yawPrevError);

  // Update previous errors
  pitchPrevError = pitchError;
  rollPrevError = rollError;
  yawPrevError = yawError;

  // Calculate PID output
  float pitchOutput = pitchP + pitchI + pitchD;
  float rollOutput = rollP + rollI + rollD;
  float yawOutput = yawP + yawI + yawD;

  // Motor control logic (adjust motor speeds based on PID output)
  motorSpeed1 = 1500 + pitchOutput + rollOutput - yawOutput;
  motorSpeed2 = 1500 + pitchOutput - rollOutput + yawOutput;
  motorSpeed3 = 1500 - pitchOutput + rollOutput + yawOutput;
  motorSpeed4 = 1500 - pitchOutput - rollOutput - yawOutput;

  // Constrain motor speeds to safe values (1000-2000)
  motorSpeed1 = constrain(motorSpeed1, 1000, 2000);
  motorSpeed2 = constrain(motorSpeed2, 1000, 2000);
  motorSpeed3 = constrain(motorSpeed3, 1000, 2000);
  motorSpeed4 = constrain(motorSpeed4, 1000, 2000);
}

// Update motor speeds
void updateMotors() {
  motor1.writeMicroseconds(motorSpeed1);
  motor2.writeMicroseconds(motorSpeed2);
  motor3.writeMicroseconds(motorSpeed3);
  motor4.writeMicroseconds(motorSpeed4);
}
