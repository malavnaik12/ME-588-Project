// Library definitions
#include "Encoder.h"
#include "Wire.h"
#include "sensorbar.h"
#include "Servo.h"
#include "SPI.h"
#include "SFE_ISL29125.h"

#define TCAADDR 0x70

// State names definitions, and associated integer values
int state = 0;
int nextState = state;
const int S0 = 0;
const int S1 = 1;
const int S2 = 2;
const int S3 = 3;
const int S4 = 4;
const int S5 = 5;
const int S6 = 6;

// Variable definitions and initialize inputs
int color_state = 0;    // set to zero at starting
int game_state = 0;     // set to zero at starting
int redpin = 23;        // pin for red led
int bluepin = 25;       // pin for blue led
int yellowpin = 27;     // pin for yellow led
int startpin = 29;      // pin for game mode led
int col_button1 = 22;   // digital output corresponding to pushbutton 1
int col_button2 = 26;   // digital output corresponding to pushbutton 2
int col_button3 = 24;   // digital output corresponding to pushbutton 3
int start_button = 28;  // digital output corresponding to start the game
int timeLimitPin = 39;  // digital output corresponding to exceeded time limit

int red_color = 0;          // Mole location square color: 0 = off, 1 = on
int yellow_color = 0;       // Mole location square color: 0 = off, 1 = on
int blue_color = 0;         // Mole location square color: 0 = off, 1 = on
int game_mode = 0;          // Game mode characterization: 0 = off, 1 = on
int color_sensor = 0;       // Simulated color sensor input: 0 = not target mole location color, 1 = target mole location color 
int ultrasonic_sensor = 0;  // Simulated ultrasonic sensor input: 0 = arena boundary not detected, 1 = arena boundary detected
int encoder = 0;            // Simulated input to execute robot turn: 0 = robot not commanded to turn, 1 = robot commanded to turn
int drop_count = 0;         // Counter to keep track of number of dropped mole whackers, counts by +1 for each successful drop, up-to 5
int lineFound = 0;          // Binary variable characterizing the instance of finding black line
int intersectCount = 0;     // Variable characterizing intersection of black lines
float startTime = 0;        // Variable used to denote the beginning of the game
float currentTime = 0;      // Time variable
int timeLimit = 0;          // Characterizes if the 2-minute time limit is exceeded
int targetColor = 0;        // Integer characterizes the target color: 1 = red, 2 = yellow, 3 = blue
int drop_confirmed = 0; 

// Motor inputs
//float trimIn = 0.5;    // Fraction of trim (between 0 and 1) used in line following
int driveIn = 55;     // Motor speed, PWM input
float inchForward = 10; // Motor speed for inching 
int driveDir = 1;     // Motor rotation direction, 1 for forward and -1 for backward
//float correctIn = 0.3;  // Fraction of motor speed, used in line following

// Sensor read variable initializations and pin definitions
  // Ultrasonic Sensor
unsigned long usPreviousTime = 0;
const int usReadDelay = 1000;
const int usEchoPin = 30;
const int usTrigPin = 6;
  // Motors
int ENA_left = 2; 
int IN1 = 3; 
int IN2 = 4;
//int encA_left = 5;
//int encB_left = 6;
int ENA_right = 7; 
int IN3 = 8; 
int IN4 = 9;
//int encA_right = 10; 
//int encB_right = 11;
  // Encoder positions for both motors
int encPos_left = 0; 
int encPos_right = 0;
  // Color Sensors
int FRcolorsensor = 2; //front right color sensor scl and sda lines on multiplexer
int BRcolorsensor = 3; //back right color sensor scl and sda lines on multiplexer
int FLcolorsensor = 4; //front left color sensor scl and sda lines on multiplexer
int BLcolorsensor = 5; //back left color sensor scl and sda lines on multiplexer
  // Dropping Servo
#include <Servo.h>
Servo leftservo;
Servo rightservo;
// Object definitions
  // Encoders
//Encoder leftEncoder(encA_left, encB_left);
//Encoder rightEncoder(encA_right, encB_right);
  // Line array
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00), for Line Sensor Array
SensorBar mySensorBar(SX1509_ADDRESS);
  // Declare color sensor object
SFE_ISL29125 RGB_sensor_1;

// Color Sensor Related Initializations
// TUNING VALUES
unsigned int redlow = 0; unsigned int redhigh = 5000000000;
unsigned int greenlow = 0; unsigned int greenhigh = 5000000000;
unsigned int bluelow = 0; unsigned int bluehigh = 5000000000;
// COLOR TUNING FOR WITHOUT WHITE LED
float redtune = .40; //proportion of total color values returned that must be red for a color to be considered red
float bluegreentune = .75; //proportion of green + blue needed to be considered blue
float redgreentune = .75; //proportion of green + red needed to be considered yellow
//COLOR TO DROP ON
char dropcolor = 'R'; //can be R, Y, or B

// Servo Related Initializations
const int servoL = 11;
const int servoR = 12;
const int servotime = 400;
const int zero = 0;
const int angle = 90;
unsigned long lastrightdrop = 0; //time of last drop on right
unsigned long lastleftdrop = 0; //time of last drop on left
int newsquareR = 1; //1 if in a square that hasn't been dropped yet
int newsquareL = 1;

// Robot action IDs
int actionID;
const int FIND_LINE = 0;
const int READ_LINE = 1;
const int GO_FORWARD = 2;
const int CORRECT_LEFT = 3;
const int CORRECT_RIGHT = 4;
const int TURN_LEFT = 5;
const int TURN_RIGHT = 6;
const int STOP = 7;

void setup() {
  // Defining output pins
  Serial.begin(115200);
  pinMode(usTrigPin,OUTPUT); pinMode(usEchoPin,INPUT);
  pinMode(col_button1, INPUT_PULLUP); pinMode(col_button2, INPUT_PULLUP);
  pinMode(col_button3, INPUT_PULLUP); pinMode(start_button, INPUT_PULLUP);
  pinMode(redpin, OUTPUT); pinMode(bluepin, OUTPUT); pinMode(yellowpin, OUTPUT);
  pinMode(startpin, OUTPUT);
  
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA_left, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENA_right, OUTPUT);

  mySensorBar.setBarStrobe();     //Default: the IR will only be turned on during reads.
  mySensorBar.clearInvertBits();  //Default: dark on light
  mySensorBar.begin();

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
  tcaselect(FRcolorsensor);
  if (RGB_sensor_1.init())  {
    Serial.println("Sensor @ Pin 2 Initialization Successful\n\r");
  }    
  tcaselect(BRcolorsensor);
  if (RGB_sensor_1.init())  {
    Serial.println("Sensor @ Pin 3 Initialization Successful\n\r");
  }
  tcaselect(FLcolorsensor);
  if (RGB_sensor_1.init())  {
    Serial.println("Sensor @ Pin 4 Initialization Successful\n\r");
  }    
  tcaselect(BLcolorsensor);
  if (RGB_sensor_1.init())  {
    Serial.println("Sensor @ Pin 5 Initialization Successful\n\r");
  }
  leftservo.attach(servoL);
  rightservo.attach(servoR);
  leftservo.write(zero);
  rightservo.write(zero);
//      delay(3000);
//      getColor(0);
//      delay(3000);
//      getColor(1);  
  Serial.println("\ndone");
}

void loop() {
  if (timeLimit == 0) {
    if (state < S2) {
      startTime = millis();
    } else {//if (state >= S3) {
      currentTime = millis();
      if ((currentTime - startTime) >= 120000) {
        driveBot(0,0,ENA_left,IN1,IN2);
        driveBot(0,0,ENA_right,IN3,IN4);
        timeLimit = 1;
        digitalWrite(timeLimitPin,HIGH);
        return;
      }
    }
    // 1. Read sensor inputs
    red_color = !digitalRead(col_button1);
    yellow_color = !digitalRead(col_button2);
    blue_color = !digitalRead(col_button3);
    game_mode = !digitalRead(start_button);
//    Serial.print("red_color"); Serial.println(red_color); 
//    Serial.print("game_mode"); Serial.println(game_mode);
//    Serial.print("game state"); Serial.println(game_state);
    Serial.print("state"); Serial.println(state);
//    Serial.print(actionID);
    // Read motor encoder postions
    //  encPos_left = leftEncoder.read(); 
    //  encPos_right = rightEncoder.read();
    //  Serial.print("Left Encoder Position: "); Serial.println(encPos_left);
    //  Serial.print("Right Encoder Position: "); Serial.println(encPos_right);
    
      
    // 2. Robot FSM
    switch(state) {
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      case S0:  // Mole square color specification
        if(red_color == HIGH && (yellow_color == LOW && blue_color == LOW) && color_state == LOW){ // condition for selecting the red color and lighting up the red led
            digitalWrite(redpin, HIGH); // lighting up the red led as a visual indicator 
            nextState = S1; // transitioning into state S1
            color_state == HIGH; // setting the new color state to High
            dropcolor = 'R';
        } else if (yellow_color == HIGH && (red_color == LOW && blue_color == LOW) && color_state == LOW){ // condition for selecting the yellow color and lighting up the yellow led
            digitalWrite(yellowpin, HIGH); // lighting up the yellow led as a visual indicator 
            nextState = S1; // transitioning into state S1
            color_state == HIGH; // setting the new color state to High
            dropcolor = 'Y';
        } else if (blue_color == HIGH && (yellow_color == LOW && red_color == LOW) && color_state == LOW){ // condition for selecting the blue color and lighting up the blue led
            digitalWrite(bluepin, HIGH); // lighting up the blue led as a visual indicator 
            nextState = S1; // transitioning into state S1
            color_state == HIGH; // setting the new color state to High
            dropcolor = 'B';            
        }
        break;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      case S1:  // Game start specification
        //digitalWrite(startpin,HIGH);
        //nextState = S2;
//        delay(500);
        if (game_mode == HIGH) {
          digitalWrite(startpin, HIGH);
          nextState = S2;
          game_state = HIGH;
        }      
//        else {
//          digitalWrite(startpin, LOW);
//          nextState = S1;
//        }
        break;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      case S2:  // Robot driving to pick up black line
        if (lineFound == 0) {
          driveBot(driveIn,1,ENA_left,IN1,IN2);
          driveBot(driveIn,1,ENA_right,IN3,IN4);
          nextState = S2;
        }
        if(mySensorBar.getDensity() > 5) { 
          driveBot(0,0,ENA_left,IN1,IN2);
          driveBot(0,0,ENA_right,IN3,IN4);
          lineFound = 1;
          actionID = STOP; 
          robotDrivingActions(actionID,ENA_left,IN1,IN2,ENA_right,IN3,IN4);
          actionID = TURN_LEFT; 
          robotDrivingActions(actionID,ENA_left,IN1,IN2,ENA_right,IN3,IN4);
          nextState = S3;
        }
        break;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      case S3:  // Robot line following
        // ROBOT DRIVING FORWARD, LINE FOLLOWING
        if (drop_count == 5) {
          Serial.println("In S3: Robot Driving Home, waiting for color sensor to detect white square");
          nextState = S6;
        } 
        Serial.println(mySensorBar.getPosition());
        if((mySensorBar.getPosition() > -25) && (mySensorBar.getPosition() < 25))  {
          actionID = GO_FORWARD; 
          robotDrivingActions(actionID,ENA_left,IN1,IN2,ENA_right,IN3,IN4); 
          nextState = S3;
        } else if( mySensorBar.getPosition() <= -25 ) {
          actionID = CORRECT_LEFT; 
          robotDrivingActions(actionID,ENA_left,IN1,IN2,ENA_right,IN3,IN4);
          nextState = S3;
        } else if( mySensorBar.getPosition() >= 25 ) {
          actionID = CORRECT_RIGHT; 
          robotDrivingActions(actionID,ENA_left,IN1,IN2,ENA_right,IN3,IN4);
          nextState = S3;
        } else {
          nextState = S3;
        }
        if(mySensorBar.getDensity() > 5) { 
          delay(400);
          intersectCount++;         
          if (intersectCount == 3) {
            actionID = STOP; robotDrivingActions(actionID,ENA_left,IN1,IN2,ENA_right,IN3,IN4);
//            nextState = S5;
//                [INSERT CODE BELOW]: Corner checking
            driveBot(inchForward,1,ENA_left,IN1,IN2);
            driveBot(inchForward,1,ENA_right,IN3,IN4);
            char detectedColor = getColor(FLcolorsensor);
            if (detectedColor == dropcolor) {
              const int trigLength = 10;
              digitalWrite(usTrigPin,HIGH); delayMicroseconds(trigLength); digitalWrite(usTrigPin,LOW);
              long duration = pulseIn(usEchoPin,HIGH);
              float distance = duration*(343/1e6)*0.5; // Units: [m], Speed of sound units: [m/us]
              if ((0.3 < distance) && (distance < 0.6)) {
                driveBot(driveIn,1,ENA_left,IN1,IN2);
                driveBot(driveIn,1,ENA_right,IN3,IN4);                
              } else if (distance <= 0.3) {
                nextState = S4;                
              }
            }
            } else {
              nextState = S5;
            }
//                get color info
//                if (color = target) {
//                  nextState = S4;
//                } else {
//                  nextState = S5;
//                }
          } else {
            nextState = S3; 
          }
//            unsigned long usCurrentTime = millis();  
//            if (usCurrentTime - usPreviousTime >= usReadDelay) {
//              const int trigLength = 10;  
//              digitalWrite(usTrigPin,HIGH); delayMicroseconds(trigLength); digitalWrite(usTrigPin,LOW);
//              long duration = pulseIn(usEchoPin,HIGH);
//              float distance = duration*(343/1e6)*0.5; // Units: [m], Speed of sound units: [m/us]
//              usPreviousTime = usCurrentTime;
//              if (0.28 < distance < 0.32) {
//                  // ultrasonic_sensor = 1; encoder = 1; 
//                  // TRANSITION TO S5 AND START TURNING
//                  nextState = S5;
//                  Serial.println("Arena Wall Detected, executing right hand turn");
//                }
//            }        
//            [INSERT CODE BELOW]: Turning after dropping a mole whacker
//            if (drop_confirmed == 1) {
//                drop_confirmed = 0;
//                nextState = S5;
//            }
        if (drop_confirmed == 1) {
            const int trigLength = 10;
            digitalWrite(usTrigPin,HIGH); delayMicroseconds(trigLength); digitalWrite(usTrigPin,LOW);
            long duration = pulseIn(usEchoPin,HIGH);
            float distance = duration*(343/1e6)*0.5; // Units: [m], Speed of sound units: [m/us]
            if ((0.3 < distance) && (distance < 0.65)) {          
              driveBot(driveIn,-1,ENA_left,IN1,IN2);
              driveBot(driveIn,-1,ENA_right,IN3,IN4);
            }
            if(mySensorBar.getDensity() > 5) { 
            delay(100);
            drop_confirmed = 0;
            intersectCount = 0;
            nextState = S5;
            }
        }
        break;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      case S4:  // Mole whacker dropping
        Serial.println("Simulated Mole Whacker Drop");
//        Serial.println("In S4: Robot deploying mole whacker while Driving, awaiting mole whacker drop confirmation");
//        if (transition4 == 1) {
//          if (serial_input == 'M') {
//            drop_count++; nextState = S3; transition4 = 0; color_sensor = 0;
//            Serial.println("Mole whacker drop confirmed, transitioning back to S3");
//          }
//        } else {
//            nextState = S4; transition4 = 1;
//        }
        break;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      case S5:  // Robot turning to continue line following
        Serial.println("In S5: Robot detected arena wall, executing right hand turn");
        actionID = STOP; robotDrivingActions(actionID,ENA_left,IN1,IN2,ENA_right,IN3,IN4);
        actionID = TURN_RIGHT; robotDrivingActions(actionID,ENA_left,IN1,IN2,ENA_right,IN3,IN4);
        nextState = S3;
        intersectCount = 0;
        break;
      case S6:  // Return home
//            unsigned long usCurrentTime = millis();  
//            if (usCurrentTime - usPreviousTime >= usReadDelay) {
//              const int trigLength = 10;  
//              digitalWrite(usTrigPin,HIGH); delayMicroseconds(trigLength); digitalWrite(usTrigPin,LOW);
//              long duration = pulseIn(usEchoPin,HIGH);
//              float distance = duration*(343/1e6)*0.5; // Units: [m], Speed of sound units: [m/us]
//              usPreviousTime = usCurrentTime;
//              if (0.28 < distance < 0.32) {
//                  // ultrasonic_sensor = 1; encoder = 1; 
//                  // TRANSITION TO S5 AND START TURNING
//                  nextState = S5;
//                  Serial.println("Arena Wall Detected, executing right hand turn");
//                }
//            }
            if((mySensorBar.getPosition() > -25) && (mySensorBar.getPosition() < 25))  {
              if(mySensorBar.getDensity() > 5) { 
                  intersectCount++;
                  delay(350);
                  if (intersectCount == 3) {
                    actionID = STOP; robotDrivingActions(actionID,ENA_left,IN1,IN2,ENA_right,IN3,IN4);
      //            [INSERT CODE BELOW]: Entering white square after objectives achieved
      //            if (any_color == white) {
      //              find where white is detected
      //              if (white == right_side) {
      //                drive forward
      //                delay(500);
      //                turn right
      //                timeLimit = 1; 
      //              } else if (white == left_side) {
      //                drive forward
      //                delay(500);
      //                turn left
      //                timeLimit = 1;                 
      //              }
      //            }                    
                  } else {
                    nextState = S6; 
                  }
                } else {
                  actionID = GO_FORWARD; robotDrivingActions(actionID,ENA_left,IN1,IN2,ENA_right,IN3,IN4); 
                }
            } else if( mySensorBar.getPosition() <= -25 ) {
              actionID = CORRECT_LEFT; robotDrivingActions(actionID,ENA_left,IN1,IN2,ENA_right,IN3,IN4);
            } else if( mySensorBar.getPosition() >= 25 ) {
              actionID = CORRECT_RIGHT; robotDrivingActions(actionID,ENA_left,IN1,IN2,ENA_right,IN3,IN4);
            } else {
              nextState = S3;
            }
        break;
    }
    state = nextState;
  } else {
    driveBot(0,0,ENA_left,IN1,IN2);
    driveBot(0,0,ENA_right,IN3,IN4);
    timeLimit = 1;
    digitalWrite(timeLimitPin,HIGH);
    delay(500);
    digitalWrite(timeLimitPin,LOW);
    delay(500);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void robotDrivingActions(int actionIDs,int ENA_left,int IN1,int IN2,int ENA_right,int IN3,int IN4) {
  float trimIn = 0.2;    // Fraction of trim (between 0 and 1) used in line following
  int driveIn = 55;     // Motor speed, PWM input
  int driveDir = 1;     // Motor rotation direction, 1 for forward and -1 for backward
  float correctIn = 0.3;  // Fraction of motor speed, used in line following
  switch (actionIDs) {
    case STOP:
      driveBot(0,0,ENA_left,IN1,IN2);
      driveBot(0,0,ENA_right,IN3,IN4);
      Serial.println("Executing STOP action");
      delay(250);
      break;
    case TURN_LEFT:
      driveBot(driveIn,1,ENA_left,IN1,IN2);
      driveBot((driveIn-driveIn)+5,-1,ENA_right,IN3,IN4);
//      delay(1000);
      delay(1250);
      break;
    case TURN_RIGHT:
      driveBot((driveIn-driveIn)+5,-1,ENA_left,IN1,IN2);
      driveBot(driveIn,1,ENA_right,IN3,IN4);
//      delay(1000);
      delay(1250);
      break;
    case GO_FORWARD:
      driveBot(driveIn,1,ENA_left,IN1,IN2);
      driveBot(driveIn,1,ENA_right,IN3,IN4);
      break;
    case CORRECT_LEFT:
      turnBot(correctIn,trimIn,ENA_left,IN1,IN2);
      turnBot(correctIn,(trimIn-trimIn),ENA_right,IN3,IN4);
      break;
    case CORRECT_RIGHT:
      Serial.println("Correcting Right");
      turnBot(correctIn,(trimIn-trimIn),ENA_left,IN1,IN2);
      turnBot(correctIn,trimIn,ENA_right,IN3,IN4);    
      break;   
  }
}

void driveBot(int driveInput,int driveDir,int pinEN,int pinIN_1,int pinIN_2) {
  analogWrite(pinEN,driveInput);
  if (driveDir == 1) {
    digitalWrite(pinIN_1, HIGH);
    digitalWrite(pinIN_2, LOW);
  } else if (driveDir == -1) {
    digitalWrite(pinIN_1, LOW);
    digitalWrite(pinIN_2, HIGH); 
  } else {
    digitalWrite(pinIN_1, LOW);
    digitalWrite(pinIN_2, LOW); 
  } 
}

void turnBot(float turnInput,float trimInput,int pinEN,int pinIN_1,int pinIN_2) {
  int driveOut;
  if(turnInput > 0 ) {
    driveOut = 255*(turnInput);
  } else {
    driveOut = 255*((-1*turnInput) + trimInput);
  }
  //Serial.println(driveOut);
  analogWrite(pinEN,driveOut);
  digitalWrite(pinIN_1, HIGH);
  digitalWrite(pinIN_2, LOW);
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

int getColor(int CS_sel) {
    tcaselect(CS_sel);
    float redIn = RGB_sensor_1.readRed();
    float greenIn = RGB_sensor_1.readGreen();
    float blueIn = RGB_sensor_1.readBlue();
    
    // Print out readings, change HEX to DEC if you prefer decimal output
    Serial.print("Color Sensor: "); Serial.println(CS_sel);

    char color = 'W';
    if((redIn/(redIn+blueIn+greenIn)) >= redtune && redIn != 0){
      color = 'R';
       Serial.println("RED");
        Serial.println((redIn/(redIn+blueIn+greenIn)));
    }

     else if((blueIn+greenIn)/(redIn+blueIn+greenIn) >= bluegreentune && blueIn != 0){
      color = 'B';
      Serial.println("BLUE");
      Serial.println((blueIn+greenIn)/(redIn+blueIn+greenIn));
    }

      else if((redIn+greenIn)/(redIn+blueIn+greenIn) >= redgreentune && greenIn != 0){
      color = 'Y';
      Serial.println("Yellow");
      Serial.println((redIn+greenIn)/(redIn+blueIn+greenIn));
    }  

    delay(20);
    return(color);
}

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
