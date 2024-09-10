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


int FR_1 = 2; 
int FR_2 = 3; 

int FL_1 = 4;
int FL_2 = 5;

int BR_1 = 6;
int BR_2 = 7; 

int BL_1 = 8;
int BL_2 = 9; 

const double KP = 0.06; //0.06  
const double KD = 0.6; //0.6
double last_error = 0;
const int goal = 3500;
int set_speed = 150;
int max_speed = 150;
int calibratePWM = 50;

// rotation control 
int rotate_speed = 35; // Speed of the rotation 
int min_speed = 1; // Minimum speed of inside wheel that creates a rotation 

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
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:

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
   if(error_control > 3500){
     delay(1000);
    }
  }

if(right_speed < min_speed){ 
    rotate_right();
    if(error_control > 3500){
      delay(1000);
    }
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
