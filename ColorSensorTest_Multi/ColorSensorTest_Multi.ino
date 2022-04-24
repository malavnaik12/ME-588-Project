// ColorSensorTest.ino -- Modified by Malav Naik: March 31, 2022
#include <Wire.h>
#include "SFE_ISL29125.h"
//#include <SPI.h>

#define TCAADDR 0x70

// Declare sensor object
SFE_ISL29125 RGB_sensor_1;

// Calibration values
unsigned int redlow = 0; unsigned int redhigh = 50000;
unsigned int greenlow = 0; unsigned int greenhigh = 50000;
unsigned int bluelow = 0; unsigned int bluehigh = 50000;

int currentVal = 0;
int numLEDs = 8;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize the ISL29125 with simple configuration so it starts sampling
  if (RGB_sensor_1.init())  {
    Serial.println("Sensor Initialization Successful\n\r");
  }
  // Initialize Multiplexer
  while (!Serial);
    delay(1000);
    Wire.begin();
    Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;
        Wire.beginTransmission(addr);
        if (!Wire.endTransmission()) {
          Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
    Serial.println("\ndone");
    delay(5000);
}

// Read sensor values for each color and print them to serial monitor
void loop() {
  getColor(4);
  getColor(7);
  // Read sensor values (16 bit integers)
//  for (int CS_select = 0; CS_select<7; CS_select++) {
//    int CS_select = 4;
//    tcaselect(CS_select);
//    unsigned int redIn = RGB_sensor_1.readRed();
//    unsigned int greenIn = RGB_sensor_1.readGreen();
//    unsigned int blueIn = RGB_sensor_1.readBlue();
//  
//    int redV = map(redIn, redlow, redhigh, 0, 255);
//    int greenV = map(greenIn, greenlow, greenhigh, 0, 255);
//    int blueV = map(blueIn, bluelow, bluehigh, 0, 255);
//  
//    int red = constrain(redV, 0, 255);
//    int green = constrain(greenV, 0, 255);
//    int blue = constrain(blueV, 0, 255);
//    
//    // Print out readings, change HEX to DEC if you prefer decimal output
//    Serial.print("Color Sensor: "); Serial.println(CS_select);
//    Serial.print("Red: "); Serial.println(red,DEC);
//    Serial.print("Green: "); Serial.println(green,DEC);
//    Serial.print("Blue: "); Serial.println(blue,DEC);
//    Serial.println();
//    delay(20);
  //}
  //delay(1000);
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void getColor(int CS_sel) {
    tcaselect(CS_sel);
    unsigned int redIn = RGB_sensor_1.readRed();
    unsigned int greenIn = RGB_sensor_1.readGreen();
    unsigned int blueIn = RGB_sensor_1.readBlue();
  
    int redV = map(redIn, redlow, redhigh, 0, 255);
    int greenV = map(greenIn, greenlow, greenhigh, 0, 255);
    int blueV = map(blueIn, bluelow, bluehigh, 0, 255);
  
    int red = constrain(redV, 0, 255);
    int green = constrain(greenV, 0, 255);
    int blue = constrain(blueV, 0, 255);
    
    // Print out readings, change HEX to DEC if you prefer decimal output
    Serial.print("Color Sensor: "); Serial.println(CS_sel);
    Serial.print("Red: "); Serial.println(red,DEC);
    Serial.print("Green: "); Serial.println(green,DEC);
    Serial.print("Blue: "); Serial.println(blue,DEC);
    Serial.println();
    delay(20);
}
