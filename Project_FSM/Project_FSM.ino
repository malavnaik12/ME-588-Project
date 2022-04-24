// Project_FSM.ino - Modified by Malav Naik on Mar 22, 2022
// modified by Harsh Pathak on April 17, 18, 19 2022
// States 0 and 1 complete

// Library definitions
#include "Encoder.h"
#include "Wire.h"
#include "sensorbar.h"

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

//FSM transition variables definitions
int transition1 = 0;
int transition2 = 0;
int transition3 = 0;
int transition4 = 0;
int transition5 = 0;
int transition6 = 0;

// Pin definitions
const int leftLS_pin = A0;
const int middleLS_pin = A1;
const int rightLS_pin = A2;
const int usEchoPin = 2;
const int usTrigPin = 3;

// Variable definitions and initialize inputs
int start_state = 0;    // set to zero at starting
int color_state = 0;    // set to zero at starting
int game_state = 0;     // set to zero at starting
int redpin = 23;        // pin for red led
int bluepin = 25;       // pin for blue led
int yellowpin = 27;     // pin for yellow led
int startpin = 29;      // pin for game mode led
int col_button1 = 22;   // digital output corresponding to pushbutton 1
int col_button2 = 24;   // digital output corresponding to pushbutton 2
int col_button3 = 26;   // digital output corresponding to pushbutton 3
int start_button = 28;  // digital output corresponding to start the game

int red_color = 0;          // Mole location square color: 0 = off, 1 = on
int yellow_color = 0;       // Mole location square color: 0 = off, 1 = on
int blue_color = 0;         // Mole location square color: 0 = off, 1 = on
int game_mode = 0;          // Game mode characterization: 0 = off, 1 = on
int line_sensor = 0;        // Simulated line sensor input: 0 = line not detected, 1 = line detected
int color_sensor = 0;       // Simulated color sensor input: 0 = not target mole location color, 1 = target mole location color 
int ultrasonic_sensor = 0;  // Simulated ultrasonic sensor input: 0 = arena boundary not detected, 1 = arena boundary detected
int encoder = 0;            // Simulated input to execute robot turn: 0 = robot not commanded to turn, 1 = robot commanded to turn
int drop_count = 0;         // Counter to keep track of number of dropped mole whackers, counts by +1 for each successful drop, up-to 5

// Sensor read variable initializations
  // Ultrasonic Sensor
unsigned long usPreviousTime = 0;
const int usReadDelay = 1000;
  // Motors
int ENA_left = 2; int IN1 = 3; int IN2 = 4;
//int encA_left = 5;
//int encB_left = 6;
int ENA_right = 7; int IN3 = 8; int IN4 = 9;
//int encA_right = 10; 
//int encB_right = 11;
  // Encoder positions for both motors
int encPos_left = 0; int encPos_right = 0;

// Object definitions
  // Encoders
//Encoder leftEncoder(encA_left, encB_left);
//Encoder rightEncoder(encA_right, encB_right);
  // Line array
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00), for Line Sensor Array
SensorBar mySensorBar(SX1509_ADDRESS);

// serial_input is used to give encoder input from the serial monitor.  
char serial_input = ' ';

void setup() {
  // Defining output pins
  Serial.begin(9600);
  pinMode(usTrigPin,OUTPUT);
  pinMode(usEchoPin,INPUT);
  pinMode(col_button1, INPUT_PULLUP);
  pinMode(col_button2, INPUT_PULLUP);
  pinMode(col_button3, INPUT_PULLUP);
  pinMode(redpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
  pinMode(yellowpin, OUTPUT);
  pinMode(start_button, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA_left, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA_right, OUTPUT);

  mySensorBar.setBarStrobe();     //Default: the IR will only be turned on during reads.
  mySensorBar.clearInvertBits();  //Default: dark on light
  mySensorBar.begin();    
  }

void loop() {
  // 1. Read sensor inputs
  red_color = !digitalRead(col_button1);
  yellow_color = !digitalRead(col_button2);
  blue_color = !digitalRead(col_button3);
  game_mode = !digitalRead(start_button);

  // Read motor encoder postions
  //  encPos_left = leftEncoder.read(); 
  //  encPos_right = rightEncoder.read();
  //  Serial.print("Left Encoder Position: "); Serial.println(encPos_left);
  //  Serial.print("Right Encoder Position: "); Serial.println(encPos_right);
  
    // Motor inputs
  int trimIn = 0.05;
  int driveIn = 50;
  int correctIn = 0.2;
    
  // 2. Robot FSM
  switch(state) {
    case S0:  // Mole square color specification
      if(red_color == HIGH && (yellow_color == LOW && blue_color == LOW) && color_state == LOW){ // condition for selecting the red color and lighting up the red led
          digitalWrite(redpin, HIGH); // lighting up the red led as a visual indicator 
          nextState = S1; // transitioning into state S1
          color_state == HIGH; // setting the new color state to High
        }
      else if(yellow_color == HIGH && (red_color == LOW && blue_color == LOW) && color_state == LOW){ // condition for selecting the yellow color and lighting up the yellow led
          digitalWrite(yellowpin, HIGH); // lighting up the yellow led as a visual indicator 
          nextState = S1; // transitioning into state S1
          color_state == HIGH; // setting the new color state to High
        }
      else if(blue_color == HIGH && (yellow_color == LOW && red_color == LOW) && color_state == LOW){ // condition for selecting the blue color and lighting up the blue led
          digitalWrite(bluepin, HIGH); // lighting up the blue led as a visual indicator 
          nextState = S1; // transitioning into state S1
          color_state == HIGH; // setting the new color state to High
        }
    break;
    case S1:  // Game start specification
      if(game_mode == HIGH && game_state == LOW){ // checking the previous state and the current state
        if(start_state == 0){ // if robot is not moving go to state 2
          nextState = S2;
          start_state = 1;
          digitalWrite(startpin, HIGH);
          }
        else{
          digitalWrite(startpin, LOW);
          start_state = 0;
          nextState = S1;
          }
        game_state = game_mode;
        delay(500);
        } else {
          digitalWrite(startpin, LOW);
          nextState = S1;
        }
      break;
    case S2:  // Robot driving to pick up black line
//      int leftLS = readLineSensor(leftLS_pin);
//      int middleLS = readLineSensor(middleLS_pin);
//      int rightLS = readLineSensor(rightLS_pin);
      //Serial.println("In S2: Robot Driving, awaiting Input for black line detection from Line Sensor");
      if (transition3 == 0) {
//        if ((leftLS > lineThreshold)&&(middleLS > lineThreshold)&&(rightLS > lineThreshold)) {
          //digitalWrite(6,1);
          // EXECUTE TURN AND BEGIN LINE FOLLOWING
          nextState = S3; transition3 = 1;
        } else {
          //line_sensor = 0; encoder = 1;
          // ROBOT CONTINUES DRIVING FORWARD, AWAITING BLACK LINE DETECTION
          nextState = S2; transition3 = 0;
        }
      if (transition3 == 1) {
        Serial.println("Black line detected, robot executes left hand turn and now following black line");
        encoder = 0;
        //digitalWrite(6,0);
      }
      break;
    case S3:  // Robot line following
      //digitalWrite(5,1);
      // ROBOT DRIVING FORWARD, LINE FOLLOWING
      // *** INSERT LINE FOLLOWING CODE HERE ***
//      leftLS = readLineSensor(leftLS_pin);
//      middleLS = readLineSensor(middleLS_pin);
//      rightLS = readLineSensor(rightLS_pin);
      if (drop_count == 5) {
        Serial.println("In S3: Robot Driving Home, waiting for color sensor to detect white square");
        nextState = S3;      
        if (serial_input == 'W') {
          nextState = S6;
          Serial.println("All mole whackers dropped, robot back at home and turning off");
        }
      } else {
          // ROBOT DRIVING FORWARD, LINE FOLLOWING AWAITING INFORMATION FROM COLOR SENSOR
          Serial.print("In S3: Robot Driving, awaiting Input for target color from Color Sensor - ");
          Serial.print("Mole Whackers Left: "); Serial.println(5-drop_count);
          if (transition4 == 0) {
            if (serial_input == 'T') { // if TARGET SQUARE COLOR IS DETECTED
              color_sensor = 1; nextState = S4; transition4 = 1; transition5 = 0; transition6 = 0;
              //digitalWrite(5,0);
              Serial.println("Target Mole Color Detected, transitioning to drop mole whacker");
            } else {
              color_sensor = 0; nextState = S3; transition4 = 0; transition5 = 0; transition6 = 0;
            }
          }
          if (transition5 == 0) {
          unsigned long currentTime = millis();  
            if (currentTime - usPreviousTime >= usReadDelay) {
              const int trigLength = 10;  
              digitalWrite(usTrigPin,HIGH); delayMicroseconds(trigLength); digitalWrite(usTrigPin,LOW);
              long duration = pulseIn(usEchoPin,HIGH);
              float distance = duration*(343/1e6)*0.5; // Units: [m], Speed of sound units: [m/us]
              usPreviousTime = currentTime;
            if (0.28 < distance < 0.32) {
                // ultrasonic_sensor = 1; encoder = 1; 
                // TRANSITION TO S5 AND START TURNING
                nextState = S5; transition4 = 0; transition5 = 1; transition6 = 0;
                digitalWrite(6,1);
                Serial.println("Arena Wall Detected, executing right hand turn");
              }
            }
          }
      }
      break;
    case S4:  // Mole whacker dropping
      Serial.println("In S4: Robot deploying mole whacker while Driving, awaiting mole whacker drop confirmation");
      if (transition4 == 1) {
        if (serial_input == 'M') {
          drop_count++; nextState = S3; transition4 = 0; color_sensor = 0;
          Serial.println("Mole whacker drop confirmed, transitioning back to S3");
        }
      } else {
          nextState = S4; transition4 = 1;
      }
      break;
    case S5:  // Robot turning to continue line following
      Serial.println("In S5: Robot detected arena wall, executing right hand turn");
      // EXECUTE TURN
//      leftLS = readLineSensor(leftLS_pin);
//      middleLS = readLineSensor(middleLS_pin);
//      rightLS = readLineSensor(rightLS_pin);
      if (transition5 == 1) { // if TURN IS COMPLETED, TRANSITION BACK TO S3
//        if ((leftLS < lineThreshold)&&(middleLS > lineThreshold)&&(rightLS < lineThreshold)) {
          nextState = S3; transition5 = 0; ultrasonic_sensor = 0; encoder = 0;
          digitalWrite(6,0);
          Serial.println("Right hand turn complete, transitioning back to S3");
      } else {
          nextState = S5; transition5 = 1;
      }
      break;
    case S6:  // Returm home
      digitalWrite(7,1);
      Serial.println("In S6: Robot at home after Mole Whackers deployed, travelling into Home square and shutting down");
      delay(1000);
      Serial.println("Mission Complete");
      nextState = S0; transition1 = 0;
      digitalWrite(2,0); digitalWrite(3,0); digitalWrite(4,0); digitalWrite(5,0); digitalWrite(6,0); digitalWrite(7,0);
      break;
  }
  state = nextState;
}
