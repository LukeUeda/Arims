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
char incomingChar = '5';

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
int min_speed = 1; // Minimum speed of inside wheel that causes a rotation (1)

// intersection logic
unsigned long lastIntersectionTime = 0; // Time when the last intersection was detected
const unsigned long debounceDelay = 2000; // Debounce delay in milliseconds
const unsigned long holdDuration = 5000; // Duration to hold intersecCount in milliseconds

unsigned long lastUpdateTime = 0; // Time when intersecCount was last updated

bool isProcessing = false; // Flag to indicate if the robot is processing an intersection
bool intersectionDetected = false; // Flag to indicate if an intersection has been processed

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

  // ------------Manual Override (no Pi)-----------
  //incomingChar = '2';

 if (incomingChar == '5'){
  stop();
 }

 if (Serial1.available() > 0) {
    // Read the incoming byte
    incomingChar = Serial1.read();

    // Print the received character to the Serial Monitor
    Serial.print("Received from raspberry pi: ");
    Serial.print(incomingChar);
    Serial.println();
  }

Serial1.println('G');

  line_follow();
  intersection_nav(incomingChar);
  
} // end of main loop

void intersection_nav(char inputChar) {
  unsigned long currentTime = millis();

  if (isProcessing) {
    return; // Skip detection if already processing an intersection
  }

  bool allSensorsAboveThreshold = true; // Assume all sensors are above threshold initially

  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] <= 500) { // Check if any sensor value is not above the threshold
      allSensorsAboveThreshold = false;
      break; // Exit loop early since not all sensors meet the condition
    }
  }

  // Set intersecFlag based on the sensor values
  if (allSensorsAboveThreshold) {
    intersecFlag = 1; // Line detected
  } else if (intersecFlag == 1) {
    // Reset flag when all sensors go below the threshold
    intersecFlag = 0;
  }

  // Process intersection if valid and debounce/hold conditions are met
  if (intersecFlag == 1 && (currentTime - lastIntersectionTime) > debounceDelay) {
    if (!intersectionDetected) {
      // Increment count and process intersection
      intersecCount++;
      stop();
      Serial.print("Intersection Count: "); // Test Print
      Serial.println(intersecCount);
      Serial.print("Input Char: ");
      Serial.println(inputChar);

      isProcessing = true; // Set processing flag to true

      if (inputChar == '1' && intersecCount == 1) {
        translate_left();
        delay(moveTime); // Wait for the specified move time
        stop(); // Stop after the movement
        lastUpdateTime = currentTime; // Update the time when the intersection count was last updated
        intersectionDetected = true; // Set flag to indicate intersection has been processed

      } else if(inputChar == '2' && intersecCount == 2){
        translate_left();
        delay(moveTime); // Wait for the specified move time
        stop(); // Stop after the movement
        lastUpdateTime = currentTime; // Update the time when the intersection count was last updated
        intersectionDetected = true; // Set flag to indicate intersection has been processed

      } else if(inputChar == '3' && intersecCount == 3){
        translate_left();
        delay(moveTime); // Wait for the specified move time
        stop(); // Stop after the movement
        lastUpdateTime = currentTime; // Update the time when the intersection count was last updated
        intersectionDetected = true; // Set flag to indicate intersection has been processed

      } else if(inputChar == '4' && intersecCount == 4){
        translate_left();
        delay(moveTime); // Wait for the specified move time
        stop(); // Stop after the movement
        lastUpdateTime = currentTime; // Update the time when the intersection count was last updated
        intersectionDetected = true; // Set flag to indicate intersection has been processed
      }

      lastIntersectionTime = currentTime; // Update the time when the last intersection was detected
    }
  }

  // Reset intersectionDetected flag after holdDuration
  if (intersectionDetected && (currentTime - lastUpdateTime) > holdDuration) {
    intersectionDetected = false; // Reset flag to allow new intersections
    intersecCount = 0; // Optionally reset intersection count
  }

  isProcessing = false; // Reset processing flag after completing the actions
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