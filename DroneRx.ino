#include <Wire.h>
#include <ESP32Servo.h>
#include <nRF24L01.h>
#include <RF24.h>

Servo motor1, motor2, motor3, motor4;
const int motor1Pin = 16;
const int motor2Pin = 17;
const int motor3Pin = 18; 
const int motor4Pin = 19;

// nRF24L01 settings
const uint64_t pipeIn = 0xE8E8F0F0E1LL; 
RF24 radio(7, 8); // CE and CSN pin

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

  // Initialize nRF24L01 radio
  if (!radio.begin()) {
    Serial.println("Radio hardware is not responding!");
    while (1) {}
  }
  
  radio.openReadingPipe(1, pipeIn);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    radio.read(&receivedData, sizeof(MyData));

    throttleControl(receivedData.throttle, receivedData.pitch, receivedData.roll, receivedData.yaw);

    applyMotorSpeeds();
  }

  delay(20);  // Short delay for loop stability
}

void throttleControl(byte throttle, byte pitch, byte roll, byte yaw) {
  // Map throttle and control inputs directly to motor speeds
  int baseSpeed = map(throttle, 0, 255, 1000, 2000);
  int pitchAdjustment = map(pitch, 0, 255, -500, 500);
  int rollAdjustment = map(roll, 0, 255, -500, 500);
  int yawAdjustment = map(yaw, 0, 255, -500, 500);

  motorSpeed1 = baseSpeed + pitchAdjustment + rollAdjustment - yawAdjustment;
  motorSpeed2 = baseSpeed + pitchAdjustment - rollAdjustment + yawAdjustment;
  motorSpeed3 = baseSpeed - pitchAdjustment + rollAdjustment + yawAdjustment;
  motorSpeed4 = baseSpeed - pitchAdjustment - rollAdjustment - yawAdjustment;
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