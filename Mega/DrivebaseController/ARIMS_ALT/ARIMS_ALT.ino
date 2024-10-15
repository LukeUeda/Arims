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
void Tread_Direction();
int intersection_nav(int intersecTarget);
void shelf_location(char rowLocation);

void thunderbirdsAreGo(char process, int row, int col);


// Wheel initialisation (1 = forward Direction, 2 = reverse direction) 
int FR_1 = 2; 
int FR_2 = 3; 
int FL_1 = 4;
int FL_2 = 5;
int BR_1 = 6;
int BR_2 = 10; // BROKEN PIN!!!!!!!!!!
int BL_1 = 8;
int BL_2 = 9;

// Serial Coms
// char incomingChar = '10';



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

char incomingChar; // Start with 'C' or 'P' for operation simulation

float scissor_angle;
float lift_height;

char previousChar = '0';







char process = '0'; // stores proccess mode. '0' = NULL, '1' = place, '2' = collect, '3' = swap
int currentIntersection = 0;



void setup() {
  // put your setup code here, to run once:

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  Serial1.begin(9600);

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
  // operation = incomingChar; // Set the operation to 'C' for collect

  // After processing 'C', simulate receiving '1' for the location
  // incomingChar = '1';

  calibrateIR(); // calibrates ir sensor (place robot on centre of line and then power up)

  // Initialize timer and set initial state
  time = millis();
  TreadDirection = HIGH; // High = retreive from shelf
  
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

while(1){
  if (Serial1.available() > 0) {
          // Read the incoming byte
          incomingChar = Serial1.read();

          if(incomingChar == 'P' || incomingChar == 'C' || incomingChar == 'S'){
          switch(incomingChar){
            case 'P': process = '1'; break;
            case 'C': process = '2'; break;
            case 'S': process = '3'; break;
            default: break;
          }
          break;
      }
    if(incomingChar == 'Z'){
      Disco();
      Stop();
    }
  }
}


while(1){
  if (Serial1.available() > 0) {
          // Read the incoming byte
          incomingChar = Serial1.read();

         if(incomingChar != 'P' || incomingChar != 'C' || incomingChar != 'S'){
          switch(incomingChar){
            case '1': row = 1; col = 3; break; // top left
            case '2': row = 1; col = 2; break; // top middle
            case '3': row = 1; col = 1; break; // top right
            case '4': row = 2; col = 3; break; // bottom left
            case '5': row = 2; col = 2; break; // bottom middle
            case '6': row = 2; col = 1; break; // bottom right
            default: break;
          }
          break;
         }

         if(incomingChar == 'Z'){
          Disco();
          Stop();
         }        
  }
}

thunderbirdsAreGo(process, row, col);
    
} // end of main loop
 


void thunderbirdsAreGo(char process, int row, int col){

  intersectFlag = 0;
  currentIntersection = 0;

  while(1){

  line_follow();

      for (int i = 0; i < SensorCount; i++) {
          if (sensorValues[i] > 500) {   // Check if each sensor value is above the threshold
            intersecFlag++;
          }
        }

        if(intersectFlag == 8){
          currentIntersection++;
          delay(50); //optional debounce to avoid multiple detections
        }

        if(currentIntersection == col){
          currentIntersection = 0;
          break;
        }

        intersectFlag = 0;
  }

  stop();
  delay(1000);

  translate_right();
  delay(moveTime); // Wait for the specified move time
  stop();

  delay(500);

  switch(row){
    case 1: Scissor_Up(1200); break;
    case 2: Scissor_Up(100); break; // change duration to better suit bottom shelf height
  }

  switch(process){
    case 'P': Tread_on(); break;
    case 'C': Tread_off(); break;
    case 'S': ; break; // add condition for Switch 
  }

  switch(row){
    case 1: Scissor_Down(1200); break;
    case 2: Scissor_Down(100); break; // change duration to better suit bottom shelf height
  }

  translate_left();
  delay(moveTime); // Wait for the specified move time
  stop();

  delay(500);

  rotate_right(4000); // adjust time delay to complete 180 degree rotation
  stop();

  delay(500);


  while(1){

  line_follow();

      for (int i = 0; i < SensorCount; i++) {
          if (sensorValues[i] > 500) {   // Check if each sensor value is above the threshold
            intersecFlag++;
          }
        }

        if(intersectFlag == 8){
          currentIntersection++;
          delay(50); //optional debounce to avoid multiple detections
        }

        if(currentIntersection == col){
          break;
        }

        intersectFlag = 0;

  }

  stop();
  delay(1000);
  rotate_right(4000); // adjust time delay to complete 180 degree rotation
  stop();

} // end of thunderbirdsAreGo




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


void Scissor_Up(const unsigned long duration) {
  if (ScissorDirection == HIGH) { 
    digitalWrite(30, LOW); // SCISSOR UP
    analogWrite(11, 0);
    delay(duration);
    analogWrite(11, 255);
    ScissorDirection = LOW;
    delay(1000);
  } 
}

void Scissor_Down(const unsigned long duration) {
  if (ScissorDirection == LOW) { 
    digitalWrite(30, HIGH); // SCISSOR UP
    analogWrite(11, 0);
    delay(duration);
    analogWrite(11, 255);
    ScissorDirection = HIGH;
    delay(1000);
  } 
}


void Tread_on(){
  digitalWrite(26, HIGH);
  analogWrite(12, 0);
  delay(1500);
  analogWrite(12, 255);
    delay(1000);
}

void Tread_off() {
  digitalWrite(26, LOW);
  analogWrite(12, 0);
  delay(1500);
  analogWrite(12, 255);
    delay(1000); 
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
  for (uint16_t i = 0; i < 200; i++) { // 200
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