// This is a test line

// Project_FSM.ino - Modified by Malav Naik on Mar 22, 2022
// modified by Harsh Pathak on April 17, 18, 2022

// Set flag for displaying the current safe status
const boolean VERBOSE = true;

// Define state names and associated integer values
const int S0 = 100;
const int S1 = 101;
const int S2 = 102;
const int S3 = 103;
const int S4 = 104;
const int S5 = 105;
const int S6 = 106;

// Pin Definitions
const int leftLS_pin = A0;
const int middleLS_pin = A1;
const int rightLS_pin = A2;
const int usEchoPin = 2;
const int usTrigPin = 3;

// Sensor Read Variable Definitions
int leftLS = 0;
int middleLS = 0;
int rightLS = 0;
unsigned long usPreviousTime = 0;
const int usReadDelay = 1000;
int lineThreshold = 800;

// Define state variable and initialize inputs
int state = S0;
int start_state = 0;
int color_state = 0;
int game_state = 0;
int redpin = 31;
int bluepin = 32;
int yellowpin = 33;
int startpin = 34;
int red_color = 0;          // Mole location square color: 0 = off, 1 = on
int yellow_color = 0;        // Mole location square color: 0 = off, 1 = on
int blue_color = 0;         // Mole location square color: 0 = off, 1 = on
int game_mode = 0;          // Game mode characterization: 0 = off, 1 = on
int line_sensor = 0;        // Simulated line sensor input: 0 = line not detected, 1 = line detected
int color_sensor = 0;       // Simulated color sensor input: 0 = not target mole location color, 1 = target mole location color 
int ultrasonic_sensor = 0;  // Simulated ultrasonic sensor input: 0 = arena boundary not detected, 1 = arena boundary detected
int encoder = 0;            // Simulated input to execute robot turn: 0 = robot not commanded to turn, 1 = robot commanded to turn
int drop_count = 0;         // Counter to keep track of number of dropped mole whackers, counts by +1 for each successful drop, up-to 5
int col_button1 = 22;       // digital output corresponding to pushbutton 1
int col_button2 = 24;       // digital output corresponding to pushbutton 2
int col_button3 = 26;       // digital output corresponding to pushbutton 3
int start_button = 30;      // digital output corresponding to start the game
// serial_input is used to give encoder input from the serial monitor.  
char serial_input = ' ';

//FSM variables
int FSM_state = 0;
int transition1 = 0;
int transition2 = 0;
int transition3 = 0;
int transition4 = 0;
int transition5 = 0;
int transition6 = 0;

void setup() {
  // Defining output pins
  Serial.begin(9600);
  pinMode(usTrigPin,OUTPUT);
  pinMode(usEchoPin,INPUT);
  pinMode(col_button1, INPUT_PULLDOWN);
  pinMode(col_button2, INPUT_PULLDOWN);
  pinMode(col_button3, INPUT_PULLDOWN);
  pinMode(col_button4, INPUT_PULLDOWN);
  pinMode(redpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
  pinMode(yellowpin, OUTPUT);
  pinMode(start_button, INPUT_PULLUP);
  }

void loop() {
  // 1. READ DEVICE INPUTS

//  if (Serial.available() > 0) { // read the incoming byte:
//    serial_input = Serial.read();
//    if (serial_input != ' ') {
//      Serial.println(serial_input);
//    }
//  }
  red_color = !digitalRead(col_button1);
  yellow_color = !digitalRead(col_button2);
  blue_color = !digitalRead(col_button3);
  game_mode = !digitalRead(start_button);
  
  switch(state) {
    case S0:  // Waiting for mole square color specification
//      if (transition1 == 0) {
//        if (serial_input == 'R') {
//          red_color = 1; green_color = 0; blue_color = 0; state = S1; transition1 = 1;
//        } else if (serial_input == 'G') {
//          red_color = 0; green_color = 1; blue_color = 0; state = S1; transition1 = 1;
//        } else if (serial_input == 'B') {
//          red_color = 0; green_color = 0; blue_color = 1; state = S1; transition1 = 1;
//        } else {
//          red_color = 0; green_color = 0; blue_color = 0; state = S0; transition1 = 0;
//        }    
////      }
//      if (transition1 == 1) {
//        if (VERBOSE) {
//          Serial.print("Mole square color ID'd: ");
//          Serial.println(serial_input);
//          Serial.println("Transitioning to S1");
//        }
//      }
//      digitalWrite(2,red_color);
//      digitalWrite(3,green_color);
//      digitalWrite(4,blue_color);
      if(red_color == HIGH && (yellow_color == LOW && blue_color == LOW) && color_state == LOW){ // condition for selecting the red color and lighting up the red led
          digitalWrite(redpin, HIGH); // lighting up the red led as a visual indicator 
          state = S1; // transitioning into state S1
          color_state == HIGH; // setting the new color state to High
        }
      else if(yellow_color == HIGH && (red_color == LOW && blue_color == LOW) && color_state == LOW){ // condition for selecting the yellow color and lighting up the yellow led
          digitalWrite(yellowpin, HIGH); // lighting up the yellow led as a visual indicator 
          state = S1; // transitioning into state S1
          color_state == HIGH; // setting the new color state to High
        }
      else if(blue_color == HIGH && (yellow_color == LOW && red_color == LOW) && color_state == LOW){ // condition for selecting the blue color and lighting up the blue led
          digitalWrite(bluepin, HIGH); // lighting up the blue led as a visual indicator 
          state = S1; // transitioning into state S1
          color_state == HIGH; // setting the new color state to High
        }
    break;
    case S1:
//      if (VERBOSE) {
//        Serial.println("In S1: Awaiting Input for Game Start");
//      }
//      if (transition2 == 0) {
//        if (serial_input == 'S') {
//          game_mode = 1; state = S2; transition2 = 1;
//        } else {
//          game_mode = 0; state = S1; transition2 = 0;
//        }      
//      }
//      if (transition2 == 1) {
//        if (VERBOSE) {
//          Serial.println("Game Started, transitioning to S2");
//        }
//      }
      if(game_mode == HIGH && game_state == LOW){ // checking the previous state and the current state
        if(start_state == 0){ // if robot is not moving go to state 2
          state = S2;
          start_state = 1;
          digitalWrite(startpin, HIGH);
          }
        else {
          digitalWrite(startpin, LOW);
          start_state = 0;
          state = S1;
          }
        }
        game_state = game_mode;
        delay(500);
      else{
          digitalWrite(startpin, LOW);
          state = S1;
        }
      break;
    case S2:
      // digitalWrite(5,1);
      // ROBOT DRIVING FORWARD, AWAITING BLACK LINE DETECTION
      int leftLS = readLineSensor(leftLS_pin);
      int middleLS = readLineSensor(middleLS_pin);
      int rightLS = readLineSensor(rightLS_pin);
      if (VERBOSE) {
        Serial.println("In S2: Robot Driving, awaiting Input for black line detection from Line Sensor");
      }
      if (transition3 == 0) {
        if ((leftLS > lineThreshold)&&(middleLS > lineThreshold)&&(rightLS > lineThreshold)) {
          //digitalWrite(6,1);
          // EXECUTE TURN AND BEGIN LINE FOLLOWING
          state = S3; transition3 = 1;
        } else {
          //line_sensor = 0; encoder = 1;
          // ROBOT CONTINUES DRIVING FORWARD, AWAITING BLACK LINE DETECTION
          state = S2; transition3 = 0;
        }
      }
      if (transition3 == 1) {
        Serial.println("Black line detected, robot executes left hand turn and now following black line");
        encoder = 0;
        //digitalWrite(6,0);
      }
      break;
    case S3:
      //digitalWrite(5,1);
      // ROBOT DRIVING FORWARD, LINE FOLLOWING
      // *** INSERT LINE FOLLOWING CODE HERE ***
      leftLS = readLineSensor(leftLS_pin);
      middleLS = readLineSensor(middleLS_pin);
      rightLS = readLineSensor(rightLS_pin);
      if (drop_count == 5) {
        Serial.println("In S3: Robot Driving Home, waiting for color sensor to detect white square");
        state = S3;      
        if (serial_input == 'W') {
          state = S6;
          Serial.println("All mole whackers dropped, robot back at home and turning off");
        }
      } else {
          // ROBOT DRIVING FORWARD, LINE FOLLOWING AWAITING INFORMATION FROM COLOR SENSOR
          Serial.print("In S3: Robot Driving, awaiting Input for target color from Color Sensor - ");
          Serial.print("Mole Whackers Left: "); Serial.println(5-drop_count);
          if (transition4 == 0) {
            if (serial_input == 'T') { // if TARGET SQUARE COLOR IS DETECTED
              color_sensor = 1; state = S4; transition4 = 1; transition5 = 0; transition6 = 0;
              //digitalWrite(5,0);
              Serial.println("Target Mole Color Detected, transitioning to drop mole whacker");
            } else {
              color_sensor = 0; state = S3; transition4 = 0; transition5 = 0; transition6 = 0;
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
                state = S5; transition4 = 0; transition5 = 1; transition6 = 0;
                digitalWrite(6,1);
                Serial.println("Arena Wall Detected, executing right hand turn");
              }
            }
          }
      }
      break;
    case S4:
      Serial.println("In S4: Robot deploying mole whacker while Driving, awaiting mole whacker drop confirmation");
      if (transition4 == 1) {
        if (serial_input == 'M') {
          drop_count++; state = S3; transition4 = 0; color_sensor = 0;
          Serial.println("Mole whacker drop confirmed, transitioning back to S3");
        }
      } else {
          state = S4; transition4 = 1;
      }
      break;
    case S5:
      Serial.println("In S5: Robot detected arena wall, executing right hand turn");
      // EXECUTE TURN
      leftLS = readLineSensor(leftLS_pin);
      middleLS = readLineSensor(middleLS_pin);
      rightLS = readLineSensor(rightLS_pin);
      if (transition5 == 1) { // if TURN IS COMPLETED, TRANSITION BACK TO S3
        if ((leftLS < lineThreshold)&&(middleLS > lineThreshold)&&(rightLS < lineThreshold)) {
          state = S3; transition5 = 0; ultrasonic_sensor = 0; encoder = 0;
          digitalWrite(6,0);
          Serial.println("Right hand turn complete, transitioning back to S3");
        }
      } else {
          state = S5; transition5 = 1;
      }
      break;
    case S6:
      digitalWrite(7,1);
      Serial.println("In S6: Robot at home after Mole Whackers deployed, travelling into Home square and shutting down");
      delay(1000);
      Serial.println("Mission Complete");
      state = S0; transition1 = 0;
      digitalWrite(2,0); digitalWrite(3,0); digitalWrite(4,0); digitalWrite(5,0); digitalWrite(6,0); digitalWrite(7,0);
      break;
  }
}

int readLineSensor(const int Pin) {
  int sensorRead = analogRead(Pin);
  return (sensorRead);
}
