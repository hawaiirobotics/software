#include <Wire.h>

const float maxAngle = 360.0;
const int fullScale = 4096;  // Full scale for 12-bit resolution

struct EncoderSettings {
  float minAngleOut;
  float maxAngleOut;
  float offset;
  int scale;
  uint8_t address;
  float lastAngle;
  float continuousAngle;
};

// Manually calibrated settings for the encoders
EncoderSettings encoders[14] = {
// arm 1
{ -90.0, 90.0, -238.18, 1, 0x40,    361, 0},
{ -90.0, 67.7, -153.0, -1, 0x41,  361, 0},
{ 6.0, 175.0, -299.0, -1, 0x43,   361, 0},
{ -180.0, 180.0, -33.0, -1, 0x42, 361, 0},
{ 0.0, 180.0, -180.0, -1, 0x45,   361, 0},
{ -180.0, 180.0, -138.8, 1, 0x44, 361, 0},
{ -144.0, 180.0, -243.46, 1, 0x46, 361, 0},
// arm 2
{ -90.0, 90.0, -332.23, 1, 0x40,    361, 0},
{ -90.0, 67.2, -258.5, -1, 0x41,  361, 0},
{ 6.0, 175.0, -280.63, -1, 0x43,   361, 0},
{ -180.0, 180.0, -26.9, -1, 0x42, 361, 0},
{ 0.0, 180.0, -256.03, -1, 0x45,   361, 0},
{ -180.0, 180.0, -63.11, 1, 0x44, 361, 0},
{ -144.0, 180.0, -161.54, -1, 0x46, 361, 0},
};

void setup() {
  Wire.begin();
  Wire.setClock(400000UL);
  Serial.begin(250000);  // Baud
  // Other setup code...
}

float mapAngle(EncoderSettings& encoder, float newAngle) {
  if (encoder.lastAngle < 361) {
    float delta = newAngle - encoder.lastAngle;

    // Check for wraparound
    if (delta > 180) {
      // Wrapped around clockwise
      encoder.continuousAngle -= (360 - delta);
    } else if (delta < -180) {
      // Wrapped around counterclockwise
      encoder.continuousAngle += (360 + delta);
    } else {
      // No wraparound
      encoder.continuousAngle += delta;
    }
  } else {
    encoder.continuousAngle = newAngle;
  }

  encoder.lastAngle = newAngle;  // Update the last angle for the next call
  float angle = encoder.continuousAngle; // use continuous angle for all other encoders
  if (encoder.address == 0x46){ //gripper
    angle = newAngle; // use raw angle for gripper encoder
  }

  float convert = 0.0;
  if (encoder.address == 0x46) { //gripper
    convert = encoder.maxAngleOut + ((angle+encoder.offset) * encoder.scale * (encoder.minAngleOut - encoder.maxAngleOut)) / 58;
  } else {
    convert = (angle + encoder.offset)*encoder.scale;
  }
  //saturators
  if (convert > encoder.maxAngleOut) {
    return encoder.maxAngleOut;
  } else if (convert < encoder.minAngleOut) {
    return encoder.minAngleOut;
  } else {
    return convert;
  }
}

void loop() {

  for (int i = 7; i < 14; i++) {
    int rawAngle = readRawAngle(encoders[i].address);

    float angle = (rawAngle / static_cast<float>(fullScale)) * maxAngle;
    float mappedAngle = mapAngle(encoders[i], angle);

    Serial.print(mappedAngle*PI/180.0);
    Serial.print(",");
  }
  Serial.println("0,0,0,0,0,0,0");

  delay(1000 / 25);  // 50Hz
}

int readRawAngle(uint8_t i2cAddress) {
  int highByte, lowByte, rawAngle;
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, 1);
  if (Wire.available() == 1) {
    highByte = Wire.read();
  } else {
    return -1;
  }
  Wire.beginTransmission(i2cAddress);
  Wire.write(0x0D);
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, 1);
  if (Wire.available() == 1) {
    lowByte = Wire.read();
  } else {
    return -1;
  }
  rawAngle = (highByte << 8) | lowByte;
  return rawAngle;
}
