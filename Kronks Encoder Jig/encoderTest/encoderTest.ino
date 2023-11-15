#include <Wire.h>
#include <Encoder.h>

Encoder CUI1(4, 5);
Encoder CUI2(6, 7);

int M1, M2, M1_P, M2_P, C1, C2;

void setup() {
  // put your setup code here, to run once:
  // Mag encoder 1
  // Mag encoder 2 (I2C)
  // CUI

  // CUI 1
  // pinMode(PIN_4, INPUT);
  // pinMode(PIN_5, INPUT);

  // // CUI 2
  // pinMode(PIN_6, INPUT);
  // pinMode(PIN_7, INPUT);

  // MAG 1
  //18 SDA
  //19 SCL
  Wire.begin();

  // MAG 2
  // 17 SDA
  // 16 SCL
  Wire1.begin();

  M1_P = 0;
  M2_P = 0;

  Serial.begin(9600);
}



void loop() {

  char buffer[50];  // Buffer to hold the formatted string
  // put your main code here, to run repeatedly:
  // AGC address 0x1A (1byte)
  // Raw Angle 0x0C (3:0) 0x0D (7:0)

  // CUI 1
  // sprintf(buffer, "C1 %ld", CUI1.read());
  // Serial.println(buffer);

  // CUI 2
  // sprintf(buffer, "C2 %ld", CUI2.read());
  // Serial.println(buffer);

  // Wire.beginTransmission(0x36);
  // Wire.write(0x1A);
  // Wire.endTransmission();
  // Wire.requestFrom(0x36, 1);
  // while(Wire.available()) {
  //   sprintf(buffer, "AGC %d", Wire.read());
  //   Serial.println(buffer);
  // }

  // Wire.beginTransmission(0x36);
  // Wire.write(0x0B);
  // Wire.endTransmission();
  // Wire.requestFrom(0x36, 1);
  // while(Wire.available()) {
  //   sprintf(buffer, "STATUS %d", Wire.read() >> 3);
  //   Serial.println(buffer);
  // }

/*
  // MAG 1
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 2);
  int M1_raw = 0;
  while(Wire.available()) {
    M1_raw = (M1_raw << 8) + Wire.read();
  }


  // MAG 2
  Wire1.beginTransmission(0x36);
  Wire1.write(0x0C);
  Wire1.endTransmission();
  Wire1.requestFrom(0x36, 2);
  int M2_raw = 0;
  while(Wire1.available()) {
    M2_raw = (M2_raw << 8) + Wire1.read();
  }


  M2_raw = -M2_raw + 2048;
  if(M2_raw < 0) M2_raw = 2*2048 + M2_raw;

  // M2_out = 1982 - M2
  M2 = M2_raw - 64;

  M1 = M1_raw - 153;

  // M1 153
  // M2 1982
  // FSR 2048
  int C1_raw, C2_raw;

  C1_raw = CUI1.read();
  C2_raw = CUI2.read();

  C1 = -C1_raw;
  C2 = C2_raw;


  // sprintf(buffer, "0 2048 %d %d %d %d", M1, M2, C1, C2);
  sprintf(buffer, "%d %d %d %d", M1, M2, C1, C2);
  // sprintf(buffer, "%d %d", M1, M2);
  // sprintf(buffer, "%d", (M1 - M2) + 63);
  Serial.println(buffer);
  delay(5);
*/
}
