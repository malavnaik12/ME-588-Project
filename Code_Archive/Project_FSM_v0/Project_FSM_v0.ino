// Project_FSM.ino - Modified by Malav Naik on Mar 22, 2022

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

// Define state variable and initialize inputs
int state = S0;
int red_color = 0;          // Mole location square color: 0 = off, 1 = on
int green_color = 0;        // Mole location square color: 0 = off, 1 = on
int blue_color = 0;         // Mole location square color: 0 = off, 1 = on
int game_mode = 0;          // Game mode characterization: 0 = off, 1 = on
int line_sensor = 0;        // Simulated line sensor input: 0 = line not detected, 1 = line detected
int color_sensor = 0;       // Simulated color sensor input: 0 = not target mole location color, 1 = target mole location color 
int ultrasonic_sensor = 0;  // Simulated ultrasonic sensor input: 0 = arena boundary not detected, 1 = arena boundary detected
int encoder = 0;            // Simulated input to execute robot turn: 0 = robot not commanded to turn, 1 = robot commanded to turn
int drop_count = 0;         // Counter to keep track of number of dropped mole whackers, counts by +1 for each successful drop, up-to 5

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
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  // Start serial port
  Serial.begin(9600);
  // Wait for serial port to be established
  delay(100);
  // Announce startup message
  if (VERBOSE) {
    Serial.println();
    Serial.println("In S0: Awaiting Mole Square Color Input");
  }
}

void loop() {
  // 1. READ DEVICE INPUTS

  if (Serial.available() > 0) { // read the incoming byte:
    serial_input = Serial.read();
    if (serial_input != ' ') {
      Serial.println(serial_input);
    }
  }

  switch(state) {
    case S0:  // Waiting for mole square color specification
      if (transition1 == 0) {
        if (serial_input == 'R') {
          red_color = 1; green_color = 0; blue_color = 0;
          state = S1;
          transition1 = 1;
        } else if (serial_input == 'G') {
          red_color = 0; green_color = 1; blue_color = 0;
          state = S1;
          transition1 = 1;
        } else if (serial_input == 'B') {
          red_color = 0; green_color = 0; blue_color = 1;
          state = S1;
          transition1 = 1;
        } else {
          red_color = 0; green_color = 0; blue_color = 0;
          state = S0;
          transition1 = 0;
        }    
      }
      if (transition1 == 1) {
        if (VERBOSE) {
          Serial.print("Mole square color ID'd: ");
          Serial.println(serial_input);
          Serial.println("Transitioning to S1");
        }
      }
      digitalWrite(2,red_color);
      digitalWrite(3,green_color);
      digitalWrite(4,blue_color);
      break;
    case S1:
      if (VERBOSE) {
        Serial.println("In S1: Awaiting Input for Game Start");
      }
      if (transition2 == 0) {
        if (serial_input == 'S') {
          game_mode = 1;
          state = S2;
          transition2 = 1;
        } else {
          game_mode = 0;
          state = S1;
          transition2 = 0;
        }      
      }
      if (transition2 == 1) {
        if (VERBOSE) {
          Serial.println("Game Started, transitioning to S2");
        }
      }
      break;
    case S2:
      digitalWrite(5,1);
      if (VERBOSE) {
        Serial.println("In S2: Robot Driving, awaiting Input for black line detection from Line Sensor");
      }
      if (transition3 == 0) {
        if (serial_input == 'L') {
          line_sensor = 1;
          encoder = 1;
          state = S3;
          digitalWrite(6,1);
          transition3 = 1;
        } else {
          line_sensor = 0;
          encoder = 1;
          state = S2;
          transition3 = 0;
        }
      }
      if (transition3 == 1) {
        Serial.println("Black line detected, robot executes left hand turn and now following black line");
        encoder = 0;
        digitalWrite(6,0);
      }
      break;
    case S3:
      digitalWrite(5,1);
      if (drop_count == 5) {
        Serial.println("In S3: Robot Driving Home, waiting for color sensor to detect white square");
        state = S3;      
        if (serial_input == 'W') {
          state = S6;
          Serial.println("All mole whackers dropped, robot back at home and turning off");
        }
      } else {
          Serial.print("In S3: Robot Driving, awaiting Input for target color from Color Sensor - ");
          Serial.print("Mole Whackers Left: "); Serial.println(5-drop_count);
          if (transition4 == 0) {
            if (serial_input == 'T') {
              color_sensor = 1;
              state = S4;
              transition4 = 1; transition5 = 0; transition6 = 0;
              digitalWrite(5,0);
              Serial.println("Target Mole Color Detected, transitioning to drop mole whacker");
            } else {
              color_sensor = 0;
              state = S3;
              transition4 = 0; transition5 = 0; transition6 = 0;
            }
          }
          if (transition5 == 0) {
            if (serial_input == 'U') {
              ultrasonic_sensor = 1;
              encoder = 1;
              state = S5;
              transition4 = 0; transition5 = 1; transition6 = 0;
              digitalWrite(6,1);
              Serial.println("Arena Wall Detected, executing right hand turn");
            }
          }
      }
      break;
    case S4:
      Serial.println("In S4: Robot deploying mole whacker while Driving, awaiting mole whacker drop confirmation");
      if (transition4 == 1) {
        if (serial_input == 'M') {
          drop_count++;
          state = S3;
          transition4 = 0;
          color_sensor = 0;
          Serial.println("Mole whacker drop confirmed, transitioning back to S3");
        }
      } else {
          state = S4;
          transition4 = 1;
      }
      break;
    case S5:
      Serial.println("In S5: Robot detected arena wall, executing right hand turn");
      if (transition5 == 1) {
        if (serial_input == 'Q') {
          state = S3;
          transition5 = 0;
          ultrasonic_sensor = 0;
          encoder = 0;
          digitalWrite(6,0);
          Serial.println("Right hand turn complete, transitioning back to S3");
        }
      } else {
          state = S5;
          transition5 = 1;
      }
      break;
    case S6:
      digitalWrite(7,1);
      Serial.println("In S6: Robot at home after Mole Whackers deployed, travelling into Home square and shutting down");
      delay(1000);
      Serial.println("Mission Complete");
      state = S0;
      transition1 = 0;
      digitalWrite(2,0); digitalWrite(3,0); digitalWrite(4,0); digitalWrite(5,0); digitalWrite(6,0); digitalWrite(7,0);
      break;
  }
}
