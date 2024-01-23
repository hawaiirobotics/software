#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

// Honolulu Attached Devices
// -------EFUSES-------
// Right Arm EFuse
//  - OC
//  - GOK
//  - EN
//  - IMON
//  - VTEMP
// Left Arm EFuse
//  - OC
//  - GOK
//  - EN
//  - IMON
//  - VTEMP
// 5V EFuse
//  - OC
//  - GOK
//  - EN
//  - IMON
//  - VTEMP
// -------DISPLAY-------
// OLED Display
//  - MOSI
//  - SCK
//  - CS
//  - RST
//  - DC
//  - MISO
// -------LIGHTING------
//  - LED_DOUT
// -------I2C Bus 0-----
// Teacher Arm Sense Addr 40h
// Lighting Sense Addr 44h
// 3v3 Sense Addr 41h
// 5v Sense Addr 45h
// -------I2C Bus 1-----
// Honolulu Extender Addr 77h
// Teacher Arm Extender Addr 76h
// Encoder Addresses TBD
// -------I2C Bus 2-----
// Honolulu Extender Addr 77h
// Teacher Arm Extender Addr 76h
// Encoder Addresses TBD


int RA_FUSE_OC = 29;
int RA_FUSE_GOK = 33;
int RA_FUSE_EN = 30;

#define RA_FUSE_IMON A12
#define RA_FUSE_VTEMP A15

int LA_FUSE_OC = 4;
int LA_FUSE_GOK = 3;
int LA_FUSE_EN = 31;

#define LA_FUSE_IMON A13
#define LA_FUSE_VTEMP A14

int FIVE_V_FUSE_OC = 35;
int FIVE_V_FUSE_GOK = 34;
int FIVE_V_FUSE_EN = 9;

#define FIVE_V_FUSE_IMON A16
#define FIVE_V_FUSE_VTEMP A17

#define TFT_CS 10
#define TFT_DC 6
#define TFT_MOSI 11
#define TFT_SCLK 13
#define TFT_RST 2
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

void setup()
{
  // TFT
  tft.init(135, 240); // Init ST7789 240x135

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Current Sense
  // 18 SDA
  // 19 SCL
  Wire.begin();

  // Teacher Arm 1
  // 17 SDA
  // 16 SCL
  Wire1.begin();

  // Teacher Arm 2
  // 25 SDA
  // 24 SCL
  Wire2.begin();

  // Right Arm EFuse
  pinMode(RA_FUSE_OC, INPUT);
  pinMode(RA_FUSE_GOK, INPUT);
  pinMode(RA_FUSE_EN, OUTPUT);

  // Left Arm EFuse
  pinMode(LA_FUSE_OC, INPUT);
  pinMode(LA_FUSE_GOK, INPUT);
  pinMode(LA_FUSE_EN, OUTPUT);

  // 5V Regulator EFuse
  pinMode(FIVE_V_FUSE_OC, INPUT);
  pinMode(FIVE_V_FUSE_GOK, INPUT);
  pinMode(FIVE_V_FUSE_EN, OUTPUT);


  //-----CURRENT SENSE CONFIGURATION-----
  // Teacher Arm Current Sense Config
  Wire.beginTransmission(0x40); // Chip addr
  // 16V FSR 0
  // PGA /4 10
  // 4 sample averaging (2.13ms) 1010
  // shunt and bus continuous 111
  // 0001 0101 0101 0111
  Wire.write((char []){0x00, 0x15, 0x57}, 3); // Reg addr
  Wire.endTransmission();

  // Lighting Current Sense Config
  Wire.beginTransmission(0x44); // Chip addr
  // 16V FSR 0
  // PGA /4 10
  // 4 sample averaging (2.13ms) 1010
  // shunt and bus continuous 111
  // 0001 0101 0101 0111
  Wire.write((char []){0x00, 0x15, 0x57}, 3); // Reg addr
  Wire.endTransmission();

  // 3v3 Current Sense Config
  Wire.beginTransmission(0x41); // Chip addr
  // 16V FSR 0
  // PGA /2 01
  // 4 sample averaging (2.13ms) 1010
  // shunt and bus continuous 111
  // 0000 1101 0101 0111
  Wire.write((char []){0x00, 0x0D, 0x57}, 3); // Reg addr
  Wire.endTransmission();

  // 5v Current Sense Config
  Wire.beginTransmission(0x45); // Chip addr
  // 16V FSR 0
  // PGA /4 10
  // 4 sample averaging (2.13ms) 1010
  // shunt and bus continuous 111
  // 0000 1101 0101 0111
  Wire.write((char []){0x00, 0x15, 0x57}, 3); // Reg addr
  Wire.endTransmission();

  //-----CURRENT SENSE CALIBRATION-----
  // Teacher Arm Current Sense Calibration
  Wire.beginTransmission(0x40); // Chip addr
  Wire.write((char []){0x05, 0x20, 0xC4}, 3); // Reg addr
  Wire.endTransmission();

  // Lighting Current Sense Calibration
  Wire.beginTransmission(0x44); // Chip addr
  Wire.write((char []){0x05, 0x22, 0xF3}, 3); // Reg addr
  Wire.endTransmission();

  // 3v3 Current Sense Calibration
  Wire.beginTransmission(0x41); // Chip addr
  Wire.write((char []){0x05, 0x41, 0x89}, 3); // Reg addr
  Wire.endTransmission();

  // 5v Current Sense Calibration
  Wire.beginTransmission(0x45); // Chip addr
  Wire.write((char []){0x05, 0x22, 0xF3}, 3); // Reg addr
  Wire.endTransmission();

  Serial.begin(115200);
}


// -------I2C Bus 0-----
// Teacher Arm Sense Addr 40h
// Lighting Sense Addr 44h
// 3v3 Sense Addr 41h
// 5v Sense Addr 45h
// -------I2C Bus 1-----
// Honolulu Extender Addr 77h
// Teacher Arm Extender Addr 76h
// Encoder Addresses TBD
// -------I2C Bus 2-----
// Honolulu Extender Addr 77h
// Teacher Arm Extender Addr 76h
// Encoder Addresses TBD
void loop()
{
  char buffer[200];  // Buffer to hold the formatted string

  //------FUSES------
  // Right Arm
  // Analog
  Serial.print("Right Arm IMON: ");
  Serial.println(analogRead(RA_FUSE_IMON));
  Serial.print("Right Arm VTEMP: ");
  Serial.println(analogRead(RA_FUSE_VTEMP));
  // Digital
  Serial.print("Right Arm OC: ");
  Serial.println(digitalRead(RA_FUSE_OC));
  Serial.print("Right Arm GOK: ");
  Serial.println(digitalRead(RA_FUSE_GOK));

  // Left Arm
  // Analog
  Serial.print("Left Arm IMON: ");
  Serial.println(analogRead(LA_FUSE_IMON));
  Serial.print("Left Arm VTEMP: ");
  Serial.println(analogRead(LA_FUSE_VTEMP));
  // Digital
  Serial.print("Left Arm OC: ");
  Serial.println(digitalRead(LA_FUSE_OC));
  Serial.print("Left Arm GOK: ");
  Serial.println(digitalRead(LA_FUSE_GOK));

  // 5V Line
  // Analog
  Serial.print("5V Line IMON: ");
  Serial.println(analogRead(FIVE_V_FUSE_IMON));
  Serial.print("5V Line VTEMP: ");
  Serial.println(analogRead(FIVE_V_FUSE_VTEMP));
  // Digital
  Serial.print("5V Line OC: ");
  Serial.println(digitalRead(FIVE_V_FUSE_OC));
  Serial.print("5V Line GOK: ");
  Serial.println(digitalRead(FIVE_V_FUSE_GOK));

  
  //------CURRENT SENSE------
  // Read Teacher Arm Current
  Wire.beginTransmission(0x40); // Chip addr
  Wire.write(0x01); // Reg addr
  Wire.endTransmission();
  Wire.requestFrom(0x40, 2);
  int TA_C = 0;
  while(Wire.available()) {
    TA_C = (TA_C << 8) + Wire.read();
  }

  // Read Teacher Arm Voltage
  Wire.beginTransmission(0x40); // Chip addr
  Wire.write(0x02); // Reg addr
  Wire.endTransmission();
  Wire.requestFrom(0x40, 2);
  int TA_V = 0;
  while(Wire.available()) {
    TA_V = (TA_V << 8) + Wire.read();
  }

  // Lighting Current
  Wire.beginTransmission(0x44); // Chip addr
  Wire.write(0x01); // Reg addr
  Wire.endTransmission();
  Wire.requestFrom(0x44, 2);
  int L_C = 0;
  while(Wire.available()) {
    L_C = (L_C << 8) + Wire.read();
  }

  // Lighting Voltage
  Wire.beginTransmission(0x44); // Chip addr
  Wire.write(0x02); // Reg addr
  Wire.endTransmission();
  Wire.requestFrom(0x44, 2);
  int L_V = 0;
  while(Wire.available()) {
    L_V = (L_V << 8) + Wire.read();
  }

  // 3v3 Current
  Wire.beginTransmission(0x41); // Chip addr
  Wire.write(0x01); // Reg addr
  Wire.endTransmission();
  Wire.requestFrom(0x41, 2);
  int THREE_C = 0;
  while(Wire.available()) {
    THREE_C = (THREE_C << 8) + Wire.read();
  }

  // 3v3 Voltage
  Wire.beginTransmission(0x41); // Chip addr
  Wire.write(0x02); // Reg addr
  Wire.endTransmission();
  Wire.requestFrom(0x41, 2);
  int THREE_V = 0;
  while(Wire.available()) {
    THREE_V = (THREE_V << 8) + Wire.read();
  }

  // 5v Current
  Wire.beginTransmission(0x45); // Chip addr
  Wire.write(0x01); // Reg addr
  Wire.endTransmission();
  Wire.requestFrom(0x45, 2);
  int FIVE_C = 0;
  while(Wire.available()) {
    FIVE_C = (FIVE_C << 8) + Wire.read();
  }

  // 5v Voltage
  Wire.beginTransmission(0x45); // Chip addr
  Wire.write(0x02); // Reg addr
  Wire.endTransmission();
  Wire.requestFrom(0x45, 2);
  int FIVE_V = 0;
  while(Wire.available()) {
    FIVE_V = (FIVE_V << 8) + Wire.read();
  }

  sprintf(buffer, "TAC %d, TAV %d, LC %d, LV %d, 3v3C %d, 3v3V %d, 5vC %d, 5vV %d", TA_C, TA_V, L_C, L_V, THREE_C, THREE_V, FIVE_C, FIVE_V);
  Serial.println(buffer);
  delay(100);
}
