#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void translate_right();
void translate_left();
void rotate_right();
void rotate_left();
void stop();
void calibrateIR();
void line_follow();
void Tread_run();
void Tread_Direction();
int intersection_nav(int intersecTarget);
void shelf_location(char rowLocation);


// Wheel initialisation (1 = forward Direction, 2 = reverse direction) 
int FR_1 = 2; 
int FR_2 = 3; 
int FL_1 = 4;
int FL_2 = 5;
int BR_1 = 6;
int BR_2 = 10; // BROKEN PIN!!!!!!!!!!
int BL_1 = 8;
int BL_2 = 9;

// Serical Coms
// char incomingChar = '10';

// General Variable
int stopFlag;   // Stop the 
int set = 1;    // Variable to prevent double counting of same intersection
int stateNumber = 0;

//Inital Values
const double KP = 0.06; //0.06  
const double KD = 0.6; //0.6
double last_error = 0;
const int goal = 3500;
int set_speed = 150;
int max_speed = 150;
int calibratePWM = 50;

// Intersections
int intersecFlag = 0;   // Flag to set if on a intersection
int intersecCount = 0;  // Keep track of the number of intersections seen 
int intersecTarget;     // Intersection number to stop at

// Shelf
int moveTime = 1500;    // Time in ms to move from line to shelf
int shelfStop = 5000;

int row = 0;
int col = 0;

// rotation control 
int rotate_speed = 35; // Speed of the rotation (35)
int min_speed = 1; // Minimum speed of inside wheel that causes a rotation (1)

// intersection logic
unsigned long lastIntersectionTime = 0; // Time when the last intersection was detected
const unsigned long debounceDelay = 2000; // Debounce delay in milliseconds
const unsigned long holdDuration = 5000; // Duration to hold intersecCount in milliseconds

unsigned long lastUpdateTime = 0; // Time when intersecCount was last updated

bool isProcessing = false; // Flag to indicate if the robot is processing an intersection
bool intersectionDetected = false; // Flag to indicate if an intersection has been processed

// Charging
const int charge_100 = 23; // digital pin corresponding to 100% charge
const int charge_75 = 25;  // digital pin corresponding to 75% charge
const int charge_50 = 27;  // digital pin corresponding to 50% charge
const int charge_25 = 29;  // digital pin corresponding to 25% charge

int charge_level = 0;     // Variable to hold the battery charge level

// Treadmill
int i = 0;
unsigned long time;
bool TreadDirection;
unsigned long start_time = 0;
float speed = 150;

bool ScissorDirection;

char operation;
char op;

int collect_flag = 0;
int place_flag = 0;

char incomingChar = 'P'; // Start with 'C' or 'P' for operation simulation

void setup() {
  // put your setup code here, to run once:
  pinMode(FR_1,OUTPUT);
  pinMode(FR_2,OUTPUT);

  pinMode(FL_1,OUTPUT);
  pinMode(FL_2,OUTPUT);

  pinMode(BR_1,OUTPUT);
  pinMode(BR_2,OUTPUT);

  pinMode(BL_1,OUTPUT);
  pinMode(BL_2,OUTPUT);

  // Initialize pin modes
  pinMode(charge_100, INPUT);
  pinMode(charge_75, INPUT);
  pinMode(charge_50, INPUT);
  pinMode(charge_25, INPUT);

  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){38, 40, 42, 44, 46, 48, 50, 52}, SensorCount);
  qtr.setEmitterPin(36);

  delay(500);
 
  pinMode(LED_BUILTIN, OUTPUT);

  // Simulate the first character input as 'C'
    operation = incomingChar; // Set the operation to 'C' for collect

  // After processing 'C', simulate receiving '1' for the location
  incomingChar = '1';

  calibrateIR(); // calibrates ir sensor (place robot on centre of line and then power up)

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  Serial1.begin(9600);

  // Initialize timer and set initial state
  time = millis();
  TreadDirection = HIGH; // High = retreive from shelf
  Serial.begin(115200);
  
  // Treadmill setup
  pinMode(26, OUTPUT);  // Direction control PIN 26
  pinMode(12, OUTPUT);  // PWM PIN 12
  analogWrite(12, 255); // Motor off (PWM is inverted)
  digitalWrite(26, LOW);

  // Scissor setup  
  ScissorDirection = HIGH;
  pinMode(30, OUTPUT);  // Direction control PIN 30
  pinMode(11, OUTPUT);  // PWM PIN 11
  analogWrite(11, 255); // Motor off (PWM is inverted)
  digitalWrite(30, HIGH);

  // Wait for the serial port to connect
  while (!Serial) {
    ;  // Wait for serial port to be ready
  }

  delay(1000);

}

void loop() {
    // ====================== Manual Override (no Pi) =============================
  
    // Update the battery status
    update_battery_status();

    // Simulate receiving 'C' first and then '1'
    if (incomingChar == 'C' || incomingChar == 'D') {
        operation = incomingChar; // Set the operation
        Serial.print("Operation set to: ");
        Serial.println(operation == 'C' ? "Collect" : "Dispense");

        // Simulate next input to be a location after operation is set
        incomingChar = '1'; // Move on to set location
    } 
    // Otherwise, check if it is a location ('1'-'9')
    else if (incomingChar >= '1' && incomingChar <= '9') {
        // Use the shelf location function to set row and column
        shelf_location(incomingChar);
        Serial.print("Location set to row: ");
        Serial.print(row);
        Serial.print(", column: ");
        Serial.println(col);

        // Proceed to move based on operation and location
        // Reset stopFlag for the new operation
        stopFlag = 0; 

        // [1] Detect if a valid shelf has been selected
        if (incomingChar == '0' || incomingChar == '10' || incomingChar == 0) {
            stop();
            stopFlag = 1;
        } else {
            // [2] Follow line until the desired intersection has been reached 
            while (intersection_nav(row) == 0 && stopFlag != 1 && stateNumber == 0) {
                line_follow();
            }
            
            // [3] Perform movement operations when the desired intersection has been reached 
            if (stopFlag != 1 && stateNumber == 1) {
                shelf_movement_right(operation); // Execute operation-specific sequence
            }
        }
    } else {
        // Reset incomingChar to stop further processing until next loop cycle
        incomingChar = 0; // Stop further processing until next loop cycle
    }
} // end if main loop

int intersection_nav(int intersecTarget){

  // Reset intersecFlag at the start of each check
  intersecFlag = 0;

  // Detect if the robot is at an intersection
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 500) {   // Check if each sensor value is above the threshold
      intersecFlag++;
    }
  }

  // Reset set flag to enable new count if not on an intersection
  if (intersecFlag < 8) {
    set = 1;   // Reset flag to enable new count
  }

  // Count the intersections and perform task depending on intersection number
  if (intersecFlag == 8 && set == 1) {  // All sensors detected the line
    intersecCount++;    // Increase intersection count
    set = 0;   // Set flag to stop double counting
    Serial.print("intersecCount: ");
    Serial.println(intersecCount);
    Serial.print("intersecTarget: ");
    Serial.println(intersecTarget);
  }

  // Determine if at goal position, return 1 if at goal, 0 if not
  if (intersecCount == intersecTarget) {
    intersecCount = 0;  // Reset intersection count
    stateNumber = 1;    // Change state if needed
    return 1;
  } else {
    return 0;
  }
}

// Function to handle collecting or dispensing based on the set operation and shelf location
void box_movement(char op, int row, int col) {
    if (op == 'C') {
        Serial.print("Collecting box from row ");
    } else if (op == 'P') {
        Serial.print("Dispensing box to row ");
    }
    Serial.print(row);
    Serial.print(", column ");
    Serial.println(col);

    // Call shelf movement based on the operation
   // shelf_movement_right(op);
}


// Finds the row and collom of the shelf that needs to be located
void shelf_location(char rowLocation){
  switch(rowLocation){
    case('1'): row = 1; col = 3; break;
    case('2'): row = 2; col = 3; break;
    case('3'): row = 3; col = 3; break;
    case('4'): row = 1; col = 2; break;
    case('5'): row = 2; col = 2; break;
    case('6'): row = 3; col = 2; break;
    case('7'): row = 1; col = 1; break;
    case('8'): row = 2; col = 1; break;
    case('9'): row = 3; col = 1; break;
    default: row = 0; col = 0; break;
  } 
}

void Scissor_run() {
  analogWrite(11, 0);
  delay(1200);
  analogWrite(11, 255);
}

void Scissor_Up() {
  if (ScissorDirection == HIGH) { 
    digitalWrite(30, LOW); // SCISSOR UP
    Scissor_run();
    ScissorDirection = LOW;
    delay(1000);
  } 
}

void Scissor_Down() {
  if (ScissorDirection == LOW) { 
    digitalWrite(30, HIGH); // SCISSOR UP
    Scissor_run();
    ScissorDirection = HIGH;
    delay(1000);
  } 
}

void Tread_run() {
  analogWrite(12, 0);
  delay(1500);
  analogWrite(12, 255);
}

void Tread_on(){
  digitalWrite(26, HIGH);
    Tread_run();
    delay(1000);
}

void Tread_off() {
  //if (TreadDirection == HIGH) {
    digitalWrite(26, LOW);
    Tread_run();
    // TreadDirection = LOW;
    delay(1500);
  //}
  
}

// Move robot towards shelf located left of the line
void shelf_movement_left(){
  translate_left();
        delay(moveTime); // Wait for the specified move time
        stop();
        Scissor_Up();
        Tread_Direction();
        Scissor_Down();
        delay(1000);
        translate_right();
        delay(moveTime);
        stop();
      stateNumber = 2;
}

// Function to handle shelf movements for both collect ('C') and dispense ('P')
void shelf_movement_right(char op) {
      translate_right();
        delay(moveTime); // Wait for the specified move time
        stop();
    // Conditional behavior based on the operation
    if (op == 'C') {
        // Collecting sequence
        Scissor_Up();    // Move the scissor lift up
        Tread_off();     // Tread gets object off shelf
        Scissor_Down();  // Move the scissor lift down
        delay(1000);     // Wait for any mechanical delays

    } else if (op == 'P') {
        // Dispensing sequence
        Scissor_Up();    // Move the scissor lift up
        Tread_on();      // Turn on the tread to push the box
        delay(1000);     // Wait to ensure the box is fully dispensed
        Scissor_Down();  // Move the scissor lift down
        delay(1000);     // Wait for any mechanical delays
    }

    // Return to the original position
    translate_left();
    delay(moveTime);
    stop();

    // Update the state number to indicate completion
    stateNumber = 2;
}

void disco(){
  translate_left();
  delay(2000);
  stop();
  rotate_right();
  Scissor_Up(); // 1 second delay
  delay(2000);
  stop();
  rotate_left();
  delay(2000);
  Scissor_Down(); // 1 second delay
  stop();
  rotate_right();
  Scissor_Up(); // 1 second delay
  delay(2000);
  stop();
  rotate_left();
  Scissor_Down(); // 1 second delay
  delay(2000);
  stop();
  translate_right();
  delay(2000);
  stop();
}

void line_follow(){

  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);
  //Serial.println(position);
  delay(10);

 int error = position - goal;
 int adjust = KP*error + KD*(error-last_error);
 last_error = error;
 
  int a = set_speed + adjust;
  int b = set_speed - adjust;

  drive_line(a,b,error);
}

void drive_line(int left_speed,int right_speed, int error_control){


  left_speed = constrain(left_speed,0,max_speed);
  right_speed = constrain(right_speed,0,max_speed);

  analogWrite(FR_1,right_speed);
  digitalWrite(FR_2,LOW);
  analogWrite(FL_1,left_speed);
  digitalWrite(FL_2,LOW);
  analogWrite(BR_1,right_speed);
  digitalWrite(BR_2,LOW);
  analogWrite(BL_1,left_speed);
  digitalWrite(BL_2,LOW);

  if(left_speed < min_speed){
    rotate_left();
  }

  if(right_speed < min_speed){ 
    rotate_right();  
  }

}

void rotate_right(){
  digitalWrite(FR_1, LOW);
  analogWrite(FR_2, rotate_speed);
  analogWrite(FL_1, rotate_speed);
  digitalWrite(FL_2, LOW);
  digitalWrite(BR_1, LOW);
  analogWrite(BR_2, rotate_speed);
  analogWrite(BL_1, rotate_speed);
  digitalWrite(BL_2, LOW);
}

void rotate_left(){
  analogWrite(FR_1, rotate_speed);
  digitalWrite(FR_2, LOW);
  digitalWrite(FL_1, LOW);
  analogWrite(FL_2, rotate_speed);
  analogWrite(BR_1, rotate_speed);
  digitalWrite(BR_2, LOW);
  digitalWrite(BL_1, LOW);
  analogWrite(BL_2, rotate_speed);
}

void translate_right(){
  digitalWrite(FR_1, LOW);
  analogWrite(FR_2, calibratePWM);
  analogWrite(FL_1, calibratePWM);
  digitalWrite(FL_2, LOW);
  analogWrite(BR_1, calibratePWM);
  digitalWrite(BR_2, LOW);
  digitalWrite(BL_1, LOW);
  analogWrite(BL_2, calibratePWM);
}

void translate_left(){
  analogWrite(FR_1, calibratePWM);
  digitalWrite(FR_2, LOW);
  digitalWrite(FL_1, LOW);
  analogWrite(FL_2, calibratePWM);
  digitalWrite(BR_1, LOW);
  analogWrite(BR_2, calibratePWM);
  analogWrite(BL_1, calibratePWM);
  digitalWrite(BL_2, LOW);
}

void stop(){
  digitalWrite(FR_1, LOW);
  digitalWrite(FR_2, LOW);
  digitalWrite(FL_1, LOW);
  digitalWrite(FL_2, LOW);
  digitalWrite(BR_1, LOW);
  digitalWrite(BR_2, LOW);
  digitalWrite(BL_1, LOW);
  digitalWrite(BL_2, LOW);
}

int update_battery_status() {
  // Check each pin and update charge level accordingly
 
  if (digitalRead(charge_100) == LOW) {
    charge_level = 100;
    //Serial1.write('A');
    Serial.println("100%");
  } 
 
  if (digitalRead(charge_75) == LOW) {
    charge_level = 75;
    //Serial1.write('B');
    Serial.println("75%");
  } 
  if (digitalRead(charge_50) == LOW) {
    charge_level = 50;
    //Serial1.write('C');
    Serial.println("50%");
  } 
  if (digitalRead(charge_25) == LOW) {
    charge_level = 25;
    //Serial1.write('D');
    Serial.println("25%");
  }
  // else {
    // If no change, resend the current charge level
    //switch (charge_level) {
      //case 100: Serial1.write('A'); break;
      //case 75: Serial1.write('B'); break;
     // case 50: Serial1.write('C'); break;
      //case 25: Serial1.write('D'); break;
    //  default: return charge_level;
    //}
  //}

  return charge_level;
}

void calibrateIR(){
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  int count = 0;
  int direction = 3;

  translate_right();
  for (uint16_t i = 0; i < 200; i++) {
    count = count + 1;

    qtr.calibrate();  // Perform sensor calibration

    if(i == 187){
      direction = 4; // random undefined direction intentionally
      stop();
      translate_right();
    }

     if ((count == 12) && (direction == 3)) {  
    direction = 1;
    count = 0;
      stop();
      translate_left();
    }

    if ((count == 25) && (direction == 0)) {  
    direction = 1;
    count = 0;
      stop();
      translate_left();
    }

  if ((count == 25) && (direction == 1)) { 
    direction = 0;
    count = 0;
      stop();
      translate_right();
    }

  }// end of for loop
  
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  stop();
  delay(1000);
}