#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Unique Address
const uint64_t pipeOut = 0xE8E8F0F0E8E8; 

RF24 radio(7, 8); // CE and CSN pins

struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
};

MyData data;  // Create a data instance

// Function to reset data to default values
void resetData() {
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
}

void setup() {
  // Start up radio communication
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  resetData();
}

int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse) {
  val = constrain(val, lower, upper);  // Ensure value stays within lower-upper range
  if (val < middle) {
    val = map(val, lower, middle, 0, 128);  // Map lower values to range 0-128
  } else {
    val = map(val, middle, upper, 128, 255);  // Map upper values to range 128-255
  }
  return (reverse ? 255 - val : val);  // Optionally reverse the output
}

void loop() {
  // Joystick Values need to be calibrated based on joysticks used
  data.throttle = mapJoystickValues(analogRead(A1), 13, 524, 1015, true);  // Inverted throttle
  data.yaw      = mapJoystickValues(analogRead(A0), 1, 505, 1020, true);   // Inverted yaw
  data.pitch    = mapJoystickValues(analogRead(A3), 12, 544, 1021, false); // Normal pitch
  data.roll     = mapJoystickValues(analogRead(A2), 34, 522, 1020, false); // Normal roll
  radio.write(&data, sizeof(MyData));
}
