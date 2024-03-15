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
// const uint8_t ARM1[] = {0x40, 0x41, 0x43, 0x42, 0x45, 0x44, 0x46};
// -------I2C Bus 2-----
// Honolulu Extender Addr 77h
// Teacher Arm Extender Addr 76h
// Encoder Addresses TBD update this
// const uint8_t ARM2[] = {0x40, 0x41, 0x43, 0x42, 0x45, 0x44, 0x46};

struct EncoderSettings {
  float minAngleOut;
  float maxAngleOut;
  float offset;
  int scale;
  uint8_t address;
  float lastAngle;
  float continuousAngle;
  float home_position;
};

// 0, 13.89, 27.77, 0, -13.89, 0,
EncoderSettings encoders[14] = {
// arm 1
{ -90.0, 90.0, -234.84, 1, 0x40,    361, 0, 0},
{ -90.0, 67.2, -155.21, -1, 0x41,  361, 0, 13.89},
{ 6.0, 175.0, -298.65, -1, 0x43,   361, 0, -27.77},
{ -180.0, 180.0, -209.27, -1, 0x42, 361, 0, 0},
{ 0.0, 180.0, -2.29, 1, 0x45,   361, 0, 13.89},
{ -180.0, 180.0, -120.59, 1, 0x44, 361, 0, 0},
{ -140.4, 157.0, -204.35, 53.87, 0x46, 361, 0, 0}, // scale is the range of the teacher arm gripper
// arm 2
{ -90.0, 90.0, -328.54, 1, 0x40,    361, 0, 0},
{ -90.0, 67.2, -261.56, -1, 0x41,  361, 0, 13.89},
{ 6.0, 175.0, -280.28, -1, 0x43,   361, 0, -27.77},
{ -180.0, 180.0, -207.07, -1, 0x42, 361, 0, 0},
{ 0.0, 180.0, -76.02, 1, 0x45,   361, 0, 13.89},
{ -180.0, 180.0, -251.28, 1, 0x44, 361, 0, 0},
{ -140.4, 157.0, -86.57, 52, 0x46, 361, 0, 0}, // scale is the range of the teacher arm gripper
};


// 0, 13.89, 27.77, 0, -13.89, 0,
// EncoderSettings encoders[14] = {
// // arm 1
// { -90.0, 90.0, -238.18, 1, 0x40,    361, 0},
// { -90.0, 67.2, -153.0, -1, 0x41,  361, 0},
// { 6.0, 175.0, -299.0, -1, 0x43,   361, 0},
// { -180.0, 180.0, -33.0, -1, 0x42, 361, 0},
// { 0.0, 180.0, -180.0, -1, 0x45,   361, 0},
// { -180.0, 180.0, -138.8, 1, 0x44, 361, 0},
// { -144.0, 180.0, -243.46, 1, 0x46, 361, 0},
// // arm 2
// { -90.0, 90.0, -332.23, 1, 0x40,    361, 0},
// { -90.0, 67.2, -258.5, -1, 0x41,  361, 0},
// { 6.0, 175.0, -280.63, -1, 0x43,   361, 0},
// { -180.0, 180.0, -26.9, -1, 0x42, 361, 0},
// { 0.0, 180.0, -256.03, -1, 0x45,   361, 0},
// { -180.0, 180.0, -63.11, 1, 0x44, 361, 0},
// { -144.0, 180.0, -161.54, -1, 0x46, 361, 0},
// };


#define TA_C_LSB 30.518e-6
#define L_C_LSB 183.11e-6
#define THREE_C_LSB 15.259e-6
#define FIVE_C_LSB 183.11e-6
#define V_LSB 0.004

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

// TP34 reset button
#define RESET_BTN A8

#define NUM_LEDS 265  // Change this to the number of LEDs in your strip
#define LIGHTING_PIN 5  // Change this to the pin number connected to your LED strip

// Initialize the NeoPixel strip
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LIGHTING_PIN, NEO_GRB + NEO_KHZ800);

void setAllLEDsToWhite() {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(255, 130, 120));
  }
  strip.show();
}

void setAllLEDsToRed() {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(255, 0, 0));
  }
  strip.show();
}

void setAllLEDsToRainbow() {
    uint32_t firstStripLength = 87;
    uint32_t secondStripLength = 89;
    uint32_t thirdStripLength = 89;
    uint32_t longestStrip = max(max(firstStripLength, secondStripLength), thirdStripLength);

    long start = millis();
    bool timeElapsed = false;
    for (long firstPixelHue = 0; firstPixelHue < 5 * 65536 && !timeElapsed; firstPixelHue += 256) {
        for (uint32_t i = 0; i < longestStrip; i++) { // For each pixel position in the longest strip...
            int pixelHue = firstPixelHue + (i * 65536L / longestStrip);
            uint32_t color = strip.gamma32(strip.ColorHSV(pixelHue));
            if (i < firstStripLength) {strip.setPixelColor(i, color);}
            if (i < secondStripLength) {strip.setPixelColor(firstStripLength + secondStripLength - 1 - i, color);}
            if (i < thirdStripLength) {strip.setPixelColor(firstStripLength + secondStripLength + i, color);}
            if (millis() - start >= 1000) {
                timeElapsed = true;
                break;
            }
        }
        strip.show();
        delay(10);
    }
}



const float p = 3.1415926;

// update frequency in Hz
#define UPDATE_FREQ 60
// update period in microseconds
#define UPDATE_RATE 1000000/UPDATE_FREQ


int readRegister(TwoWire w, int chip_addr, int reg_addr, int length) {
    int result = 0;
    
    w.beginTransmission(chip_addr);
    w.write(reg_addr);
    w.endTransmission();
    w.requestFrom(chip_addr, length);

    if (!w.available()) {  
        return -1;
    }

    while (w.available()) {
        result = (result << 8) + w.read();
    }

    return result;
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
    convert = encoder.maxAngleOut + ((angle+encoder.offset) * (encoder.minAngleOut - encoder.maxAngleOut)) / encoder.scale;
  } else {
    convert = (angle + encoder.offset)*encoder.scale;
  }
  if (convert > encoder.maxAngleOut) {
    return encoder.maxAngleOut;
  } else if (convert < encoder.minAngleOut) {
    return encoder.minAngleOut;
  } else {
    return convert;
  }
}

// print offsets if the arms need to be permanently updated
void print_offsets() {
  char buffer[200];

  strncat(buffer, "SO,", 3);
  for(int i = 0; i < 14; i++) {
    sprintf(buffer + strlen(buffer), "%6.2f,", encoders[i].offset);
  }
  strncat(buffer, "EO", 2);

  Serial.println(buffer);
}

// update the offset for a new zero position
void update_offset() {
    float rawAngle;

    // no need to update gripper offsets

    for(int i = 0; i < 6; i++) {
      rawAngle = (readRegister(Wire2, encoders[i].address, 0x0C, 2) / 4096.0 * 360.0);
      encoders[i].offset = encoders[i].home_position - rawAngle;
    }

    for(int i = 7; i < 13; i++) {
      rawAngle = (readRegister(Wire1, encoders[i].address, 0x0C, 2) / 4096.0 * 360.0);
      encoders[i].offset = encoders[i].home_position - rawAngle;
    }
}

void EncoderErrorReadings() {
  setAllLEDsToRed();  
  exit(-1);
}

void setup()
{
  strip.begin();
  strip.show(); 
  setAllLEDsToWhite();

  //-------TFT-------
  tft.init(135, 240); // Init ST7789 240x135

  // tft rotation
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  //---RESET BUTTON---
  pinMode(RESET_BTN, INPUT_PULLUP);

  //-------I2C-------
  // Current Sense
  // 18 SDA
  // 19 SCL
  Wire.begin();

  // Teacher Arm 1
  // 17 SDA
  // 16 SCL
  Wire1.begin();
  Wire1.setClock(400000UL);

  // Teacher Arm 2
  // 25 SDA
  // 24 SCL
  Wire2.begin();
  Wire2.setClock(400000UL);

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
  Serial.begin(250000);
}

void loop()
{

    uint32_t startTime = micros(); // Record the start time

    static uint8_t reset_debounce = 0;
    
    // update the offsets when the reset button is held for 1s
    if(reset_debounce >= 30) {
      Serial.println("REGARDS");
      // exit(1);
      update_offset();
      setAllLEDsToRainbow();
      setAllLEDsToWhite();
      reset_debounce = 0;
    } else if(digitalRead(RESET_BTN) == LOW ) {
      reset_debounce++;
    } else {
      reset_debounce = 0;
    }

    // print_offsets();

    static uint8_t counter = 0;
    char buffer[200];
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

        // Read Teacher Arm Voltage
    // Wire.beginTransmission(0x40); // Chip addr
    // Wire.write(0x02); // Reg addr
    // Wire.endTransmission();
    // Wire.requestFrom(0x40, 2);
    // int TA_V = 0;
    // while(Wire.available()) {
    //   TA_V = (TA_V << 8) + Wire.read();
    // }


    //------CURRENT SENSE------
    // Read Teacher Arm Current
    uint16_t RAW_TA_C = readRegister(Wire, 0x40, 0x04, 2);
    float TA_C = RAW_TA_C * TA_C_LSB;

    // Read Teacher Arm Voltage
    uint16_t RAW_TA_V = readRegister(Wire, 0x40, 0x02, 2) >> 3;
    float TA_V = RAW_TA_V * V_LSB;

    // Lighting Current
    // LSB 183.11e-6
    uint16_t RAW_L_C = readRegister(Wire, 0x44, 0x04, 2);
    float L_C = RAW_L_C * L_C_LSB;

    // Lighting Voltage
    uint16_t RAW_L_V = readRegister(Wire, 0x44, 0x02, 2) >> 3;
    float L_V = RAW_L_V * V_LSB;

    // 3v3 Current
    // LSB 15.259e-6
    uint16_t RAW_THREE_C = readRegister(Wire, 0x41, 0x04, 2);
    float THREE_C = RAW_THREE_C * THREE_C_LSB;

    // 3v3 Voltage
    uint16_t RAW_THREE_V = readRegister(Wire, 0x41, 0x02, 2) >> 3;
    float THREE_V = RAW_THREE_V * V_LSB;

    // 5v Current
    // LSB 183.11e-6
    uint16_t RAW_FIVE_C = readRegister(Wire, 0x45, 0x04, 2);
    float FIVE_C = RAW_FIVE_C * FIVE_C_LSB;

    // 5v Voltage
    uint16_t RAW_FIVE_V = readRegister(Wire, 0x45, 0x02, 2) >> 3;
    float FIVE_V = RAW_FIVE_V * V_LSB;

    // memset(buffer, 0, 200);
    // sprintf(buffer, "TAC %f, TAV %d, LC %f, LV %f, 3v3C %f, 3v3V %f, 5vC %f, 5vV %f", TA_C, RAW_TA_V, L_C, L_V, THREE_C, THREE_V, FIVE_C, FIVE_V);
    // Serial.println(buffer);

    //--------TEACHER ARM JOINT ANGLES-------
    float arm1_joint_angles[7];
    float arm2_joint_angles[7];

    float rawAngle = 0.0;

    for(int i = 0; i < 7; i++) {
      rawAngle = (readRegister(Wire2, encoders[i].address, 0x0C, 2) / 4096.0 * 360.0);
      arm1_joint_angles[i] = mapAngle(encoders[i], rawAngle)*PI/180.0;
      if(rawAngle < 0){
        EncoderErrorReadings();
      }
    }

    for(int i = 7; i < 14; i++) {
      rawAngle = (readRegister(Wire1, encoders[i].address, 0x0C, 2) / 4096.0 * 360.0);
      arm2_joint_angles[i-7] = mapAngle(encoders[i], rawAngle)*PI/180.0;
      if(rawAngle < 0){
        EncoderErrorReadings();
      }
    }

    // Send ARM1 angles back over serial
    memset(buffer, 0, 200);
    // strncat(buffer, "SA,", 3);
    for(int i = 0; i < 7; i++) {
      sprintf(buffer + strlen(buffer), "%6.2f,", arm1_joint_angles[i]);
    }
    // strncat(buffer, "EA", 2);

    // Send ARM2 angles back over serial
    // memset(buffer, 0, 600);
    // strncat(buffer, "SB,", 4);
    for(int i = 0; i < 7; i++) {
      if(i==6){
        sprintf(buffer + strlen(buffer), "%6.2f", arm2_joint_angles[i]);
      }
      else{
        sprintf(buffer + strlen(buffer), "%6.2f,", arm2_joint_angles[i]);
      }
    }
    // strncat(buffer, "EB", 2);

    Serial.println(buffer);

    if(counter == 5){
      counter = 0;

      tft.setTextWrap(true);
      tft.setCursor(0, 0);
      // tft.fillScreen(ST77XX_BLACK);
      // tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setTextColor(ST77XX_WHITE);
      tft.setTextSize(2);

      memset(buffer, 0, 200);
      sprintf(buffer, "TAC %4.2f, TAV %4.2f\nLC %4.2f, LV %4.2f\n3v3C %4.2f, 3v3V %4.2f\n5vC %4.2f, 5vV %4.2f", TA_C, TA_V, L_C, L_V, THREE_C, THREE_V, FIVE_C, FIVE_V);

      tft.print(buffer);
    } else {
      counter++;
    }

    delayMicroseconds(UPDATE_RATE - (micros() - startTime));
    // float frequency = 1000000/(micros() - startTime);
    // Serial.print("F = ");
    // Serial.println(frequency);
}