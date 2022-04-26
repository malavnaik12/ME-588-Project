 // ColorSensorTest.ino -- Modified by Malav Naik: March 31, 2022
#include <Wire.h>
#include "SFE_ISL29125.h"
//#include <SPI.h>

#define TCAADDR 0x70

//COLOR TO DROP ON
char dropcolor = 'R'; //can be R, Y, or B

//SERVO RELATED VALUES
#include <Servo.h>
Servo leftservo;
Servo rightservo;
const int servoL = 5;
const int servoR = 6;
const int servotime = 400;
const int zero = 0;
const int angle = 90;
unsigned long lastrightdrop = 0; //time of last drop on right
unsigned long lastleftdrop = 0; //time of last drop on left
int newsquareR = 1; //1 if in a square that hasn't been dropped yet
int newsquareL = 1;


// Declare sensor object
SFE_ISL29125 RGB_sensor_1;

int FRcolorsensor = 0; //front right color sensor scl and sda lines on multiplexer
int BRcolorsensor = 1; //back right color sensor scl and sda lines on multiplexer
int FLcolorsensor = 5; //front left color sensor scl and sda lines on multiplexer
int BLcolorsensor = 6; //back left color sensor scl and sda lines on multiplexer

//TUNING VALUES
unsigned int redlow = 0; unsigned int redhigh = 5000000000;
unsigned int greenlow = 0; unsigned int greenhigh = 5000000000;
unsigned int bluelow = 0; unsigned int bluehigh = 5000000000;


float redtune = .60; //proportion of total color values returned that must be red for a color to be considered red
float bluegreentune = .75; //proportion of green + blue needed to be considered blue
float redgreentune = .75; //proportion of green + red needed to be considered yellow


int currentVal = 0;
int numLEDs = 8;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Initialize serial communication
  Serial.begin(9800);

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
  
  //SERVO
  leftservo.attach(servoL);
  rightservo.attach(servoR);
  leftservo.write(zero);
  rightservo.write(zero);

  
    delay(50);
    
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Read sensor values for each color and print them to serial monitor
void loop() {
  checkDrop(dropcolor);
  delay(5);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int getColor(int CS_sel) {
    tcaselect(CS_sel);
    unsigned int redIn = RGB_sensor_1.readRed();
    unsigned int greenIn = RGB_sensor_1.readGreen();
    unsigned int blueIn = RGB_sensor_1.readBlue();
  
    int redV = map(redIn, redlow, redhigh, 0, 255);
    int greenV = map(greenIn, greenlow, greenhigh, 0, 255);
    int blueV = map(blueIn, bluelow, bluehigh, 0, 255);
  
    float red = constrain(redV, 0, 255);
    float green = constrain(greenV, 0, 255);
    float blue = constrain(blueV, 0, 255);
    
    // Print out readings, change HEX to DEC if you prefer decimal output
    Serial.print("Color Sensor: "); Serial.println(CS_sel);
    Serial.print("Red: "); Serial.println(red,DEC);
    
    Serial.print("Green: "); Serial.println(green,DEC);
    Serial.print("Blue: "); Serial.println(blue,DEC);
    Serial.println();

    char color = 'W';
    if((red/(red+blue+green)) >= redtune && red != 0){
      color = 'R';
    }

     else if((blue+green)/(red+blue+green) >= bluegreentune && blue != 0){
      color = 'B';
    }

      else if((red+green)/(red+blue+green) >= redgreentune && green != 0){
      color = 'Y';
    }  

    delay(20);
    return(color);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void checkDrop(int COI) { //accepts color of interest

    char FR_RGB = getColor(FRcolorsensor);
    char BR_RGB = getColor(BRcolorsensor);
    char FL_RGB = getColor(FLcolorsensor);
    char BL_RGB = getColor(BLcolorsensor);

    if(FR_RGB == BR_RGB && FR_RGB == COI && newsquareR == 1){
    Serial.println("Drop Right");
    rightservo.write(angle);
    lastrightdrop = millis();
    newsquareR = 0;
    }
    
    if(FL_RGB == BL_RGB && FL_RGB == COI && newsquareL == 1){
      Serial.println("Drop Left");
      leftservo.write(angle);
      lastleftdrop = millis();
      newsquareL = 0;
    }
    
    if(newsquareR == 0 && millis()-lastrightdrop >= servotime){
    rightservo.write(zero);
    }
    
    if(newsquareL == 0 && millis()-lastleftdrop >= servotime){
    leftservo.write(zero);
    }

}
