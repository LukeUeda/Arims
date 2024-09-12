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
int intersection_nav(int intersecTarget);

// Wheel initialisation (1 = forward Direction, 2 = reverse direction) 
int FR_1 = 2; 
int FR_2 = 3; 
int FL_1 = 4;
int FL_2 = 5;
int BR_1 = 6;
int BR_2 = 7; 
int BL_1 = 8;
int BL_2 = 9;

// Serical Coms
char receivedChar;
char incomingChar;

// General Variable
int stopFlag;   // Stop the 

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
int moveTime = 2000;    // Time in ms to move from line to shelf

// rotation control 
int rotate_speed = 35; // Speed of the rotation (35)
int min_speed = 1; // Minimum speed of inside wheel that creates a rotation (1)

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

  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){38, 40, 42, 44, 46, 48, 50, 52}, SensorCount);
  qtr.setEmitterPin(36);

  delay(500);
 
  pinMode(LED_BUILTIN, OUTPUT);

  calibrateIR(); // calibrates ir sensor (place robot on centre of line and then power up)

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  Serial1.begin(9600);

// Wait for the serial port to connect
  while (!Serial) {
    ;  // Wait for serial port to be ready
  }

  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:

 if (Serial1.available() > 0) {
    // Read the incoming byte
    incomingChar = Serial1.read();

    // Print the received character to the Serial Monitor
    Serial.print("Received from raspberry pi: ");
    Serial.print(incomingChar);
    Serial.println();
  }


  // MANUAL OVERRIDE (comment out to disable)
  // incomingChar = 2;

  // [1] Wait for input from raspberry Pi before starting
  if(incomingChar == '9' || incomingChar == 0){   // Halt conditions
    stop();
    stopFlag = 1;
  }
  else{

    // [2] Follow line until desired intersection is reached
    while(intersection_nav(incomingChar) == 0 && stopFlag != 1){
      line_follow();
    }

    // [3] Move off the line
    while (millis() < millis()+moveTime && stopFlag != 1){
      translate_left();
    }
  }

} // end of main loop

int intersection_nav(int intersecTarget){

  // function Variables 
  int set = 1;    // Variable to prevent double counting of same intersection

  // Detect if robot is at an intersection
  for(int i = 0; i < SensorCount; i++){
    if(sensorValues[i] > 500){    // All sensor values seeing relativly same value 
      intersecFlag = 1;
    }else{
      intersecFlag = 0;
      set = 1;    // reset flag to enable new count  
    }
  }

  // Count the intersections and perform task depending on intersection number
  if(intersecFlag == 1 && set == 1){    // if at new intersection
    intersecCount = intersecCount + 1;    // Increase intersection count
    set = 0;    // Set flag to stop double counting    
  }

  // Determin if at goal position, return 1 if at goal, 0 if not
  if(intersecCount == intersecTarget){
    return 1;
  }
  else{
    return 0;
  }
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