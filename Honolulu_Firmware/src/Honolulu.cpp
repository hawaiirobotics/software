#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <Adafruit_NeoPixel.h>


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
// Encoder Addresses 
// J1 0x20
// J2 0x41
// J3 0x43
// J4 0x42
// J5 0x45
// J6 0x44
// J7 0x46
const uint8_t ARM1[] = {0x20, 0x41, 0x43, 0x42, 0x45, 0x44, 0x46};
// -------I2C Bus 2-----
// Honolulu Extender Addr 77h
// Teacher Arm Extender Addr 76h
// Encoder Addresses TBD update this
const uint8_t ARM2[] = {0x20, 0x41, 0x43, 0x42, 0x45, 0x44, 0x46};

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
// Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);


#define LIGHTING 5
#define NUM_LEDS 10
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LIGHTING, NEO_GRB + NEO_KHZ800);

const float p = 3.1415926;


void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

int readRegister(TwoWire w, int chip_addr, int reg_addr, int length) {
    int result = 0;
    
    w.beginTransmission(chip_addr);
    w.write(reg_addr);
    w.endTransmission();
    w.requestFrom(chip_addr, length);

    while(w.available()) {
        result = (result << 8) + w.read();
    }

    return result;
}

void tftPrintTest() {
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(4);
  tft.print(1234.567);
  delay(1500);
  tft.setCursor(0, 0);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(0);
  tft.println("Hello World!");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.print(p, 6);
  tft.println(" Want pi?");
  tft.println(" ");
  tft.print(8675309, HEX); // print 8,675,309 out in HEX!
  tft.println(" Print HEX!");
  tft.println(" ");
  tft.setTextColor(ST77XX_WHITE);
  tft.println("Sketch has been");
  tft.println("running for: ");
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print(millis() / 1000);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(" seconds.");
}

void setup()
{

  //-------TFT-------
  tft.init(135, 240); // Init ST7789 240x135

  // tft rotation
  tft.setRotation(3);

  tft.fillScreen(ST77XX_BLACK);
  delay(500);

  // large block of text
  tft.fillScreen(ST77XX_BLACK);
  testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST77XX_WHITE);
  delay(1000);

  tftPrintTest();
  delay(1000);

  //-------I2C-------
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
  Wire2.setClock( 50000UL );

  //-------FUSE GPIO--------
  // Right Arm EFuse
  pinMode(RA_FUSE_OC, INPUT);
  pinMode(RA_FUSE_GOK, INPUT);
  pinMode(RA_FUSE_EN, OUTPUT);
  digitalWrite(RA_FUSE_EN, HIGH);

  // Left Arm EFuse
  pinMode(LA_FUSE_OC, INPUT);
  pinMode(LA_FUSE_GOK, INPUT);
  pinMode(LA_FUSE_EN, OUTPUT);
  digitalWrite(LA_FUSE_EN, HIGH);

  // 5V Regulator EFuse
  pinMode(FIVE_V_FUSE_OC, INPUT);
  pinMode(FIVE_V_FUSE_GOK, INPUT);
  pinMode(FIVE_V_FUSE_EN, OUTPUT);
  digitalWrite(FIVE_V_FUSE_EN, HIGH);


  //-----CURRENT SENSE CONFIGURATION-----
  // Teacher Arm Current Sense Config
  Wire.beginTransmission(0x40); // Chip addr
  // 16V FSR 0
  // PGA /4 10
  // 4 sample averaging (2.13ms) 1010
  // shunt and bus continuous 111
  // 0001 0101 0101 0111
  char curr_sns_cfg1 [3] = {0x00, 0x15, 0x57};
  Wire.write(curr_sns_cfg1, 3); // Reg addr
  Wire.endTransmission();

  // Lighting Current Sense Config
  Wire.beginTransmission(0x44); // Chip addr
  // 16V FSR 0
  // PGA /4 10
  // 4 sample averaging (2.13ms) 1010
  // shunt and bus continuous 111
  // 0001 0101 0101 0111
  Wire.write(curr_sns_cfg1, 3); // Reg addr
  Wire.endTransmission();

  // 3v3 Current Sense Config
  Wire.beginTransmission(0x41); // Chip addr
  // 16V FSR 0
  // PGA /2 01
  // 4 sample averaging (2.13ms) 1010
  // shunt and bus continuous 111
  // 0000 1101 0101 0111
  char curr_sns_cfg2 [3] = {0x00, 0x0D, 0x57};
  Wire.write(curr_sns_cfg2, 3); // Reg addr
  Wire.endTransmission();

  // 5v Current Sense Config
  Wire.beginTransmission(0x45); // Chip addr
  // 16V FSR 0
  // PGA /4 10
  // 4 sample averaging (2.13ms) 1010
  // shunt and bus continuous 111
  // 0000 1101 0101 0111
  Wire.write(curr_sns_cfg1, 3); // Reg addr
  Wire.endTransmission();



  //-----CURRENT SENSE CALIBRATION-----
  // Teacher Arm Current Sense Calibration
  Wire.beginTransmission(0x40); // Chip addr
  char curr_sns_cal1 [3] = {0x05, 0x20, 0xC4};
  Wire.write(curr_sns_cal1, 3); // Reg addr
  Wire.endTransmission();

  // Lighting Current Sense Calibration
  Wire.beginTransmission(0x44); // Chip addr
  char curr_sns_cal2 [3] = {0x05, 0x22, 0xF3};
  Wire.write(curr_sns_cal2, 3); // Reg addr
  Wire.endTransmission();

  // 3v3 Current Sense Calibration
  Wire.beginTransmission(0x41); // Chip addr
  char curr_sns_cal3 [3] ={0x05, 0x41, 0x89};
  Wire.write(curr_sns_cal3, 3); // Reg addr
  Wire.endTransmission();

  // 5v Current Sense Calibration
  Wire.beginTransmission(0x45); // Chip addr
  Wire.write(curr_sns_cal2, 3); // Reg addr
  Wire.endTransmission();

  //setup serial
  Serial.begin(115200);
}

void loop()
{
    //------FUSES------
    // Right Arm
    // Analog
    // Serial.print("Right Arm IMON: ");
    // Serial.println(analogRead(RA_FUSE_IMON));
    // Serial.print("Right Arm VTEMP: ");
    // Serial.println(analogRead(RA_FUSE_VTEMP));
    // Digital
    // Serial.print("Right Arm OC: ");
    // Serial.println(digitalRead(RA_FUSE_OC));
    // Serial.print("Right Arm GOK: ");
    // Serial.println(digitalRead(RA_FUSE_GOK));

    // Left Arm
    // Analog
    // Serial.print("Left Arm IMON: ");
    // Serial.println(analogRead(LA_FUSE_IMON));
    // Serial.print("Left Arm VTEMP: ");
    // Serial.println(analogRead(LA_FUSE_VTEMP));
    // Digital
    // Serial.print("Left Arm OC: ");
    // Serial.println(digitalRead(LA_FUSE_OC));
    // Serial.print("Left Arm GOK: ");
    // Serial.println(digitalRead(LA_FUSE_GOK));

    // 5V Line
    // Analog
    // Serial.print("5V Line IMON: ");
    // Serial.println(analogRead(FIVE_V_FUSE_IMON));
    // Serial.print("5V Line VTEMP: ");
    // Serial.println(analogRead(FIVE_V_FUSE_VTEMP));
    // Digital
    // Serial.print("5V Line OC: ");
    // Serial.println(digitalRead(FIVE_V_FUSE_OC));
    // Serial.print("5V Line GOK: ");
    // Serial.println(digitalRead(FIVE_V_FUSE_GOK));

    
    //------CURRENT SENSE------
    // Read Teacher Arm Current
    int TA_C = readRegister(Wire, 0x40, 0x01, 2);

    // Read Teacher Arm Voltage
    int TA_V = readRegister(Wire, 0x40, 0x02, 2);

    // Lighting Current
    int L_C = readRegister(Wire, 0x44, 0x01, 2);

    // Lighting Voltage
    int L_V = readRegister(Wire, 0x44, 0x02, 2);

    // 3v3 Current
    int THREE_C = readRegister(Wire, 0x41, 0x01, 2);

    // 3v3 Voltage
    int THREE_V = readRegister(Wire, 0x41, 0x02, 2);

    // 5v Current
    int FIVE_C = readRegister(Wire, 0x45, 0x01, 2);

    // 5v Voltage
    int FIVE_V = readRegister(Wire, 0x45, 0x02, 2);

    // char buffer[200];
    // sprintf(buffer, "TAC %d, TAV %d, LC %d, LV %d, 3v3C %d, 3v3V %d, 5vC %d, 5vV %d", TA_C, TA_V, L_C, L_V, THREE_C, THREE_V, FIVE_C, FIVE_V);
    // Serial.println(buffer);

    //--------TEACHER ARM JOINT ANGLES-------
    float arm1_joint_angles[7];
    float arm2_joint_angles[7];


    for(int i = 0; i < 7; i++) {
      arm1_joint_angles[i] = (readRegister(Wire1, ARM1[i], 0x0C, 2) / 4096) * TWO_PI;
    }

    for(int i = 0; i < 7; i++) {
      arm2_joint_angles[i] = (readRegister(Wire2, ARM2[i], 0x0C, 2) / 4096) * TWO_PI;
    }

    // Send ARM1 angles back over serial
    char buffer[200];
    strncat(buffer, "SA,", 3);
    for(int i = 0; i < 7; i++) {
      sprintf(buffer + strlen(buffer), "%f,", arm1_joint_angles[i]);
    }
    strncat(buffer, "EA", 2);

    Serial.println(buffer);

    // Send ARM2 angles back over serial
    char buffer[200];
    strncat(buffer, "SB,", 4);
    for(int i = 0; i < 7; i++) {
      sprintf(buffer + strlen(buffer), "%f,", arm2_joint_angles[i]);
    }
    strncat(buffer, "EB", 2);

    Serial.println(buffer);

    tftPrintTest();
    delay(100);
}