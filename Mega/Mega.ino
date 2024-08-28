#include "./Motor.hpp"

int ia1 = 2;
int ib1 = 3;

int ia2 = 4;
int ib2 = 5;

int ia3 = 6;
int ib3 = 7;

int ia4 = 8;
int ib4 = 9;

int ch1 = 10;
int ch2 = 11;
int ch3 = 12;
int ch4 = 13;

int ch1_pulse;
int ch2_pulse;
int ch3_pulse;
int ch4_pulse;

float ch1_power;
float ch2_power;
float ch3_power;
float ch4_power;

float m1_duty;
float m2_duty;
float m3_duty;
float m4_duty;

Motor m1(ia1, ib1, 0, 0);
Motor m2(ia2, ib2, 0, 0);
Motor m3(ia3, ib3, 0, 0);
Motor m4(ia4, ib4, 0, 0);

String command;

void setup(){
    m1.setControlMode(DUTY_CYCLE_CONTROL);
    m2.setControlMode(DUTY_CYCLE_CONTROL);
    m3.setControlMode(DUTY_CYCLE_CONTROL);
    m4.setControlMode(DUTY_CYCLE_CONTROL);
    m3.reverse = true;

    pinMode(ch1, INPUT);
    pinMode(ch2, INPUT);
    pinMode(ch3, INPUT);
    pinMode(ch4, INPUT);

    Serial.begin(9600);
}

void loop(){
  ch1_pulse = pulseIn(ch1, HIGH);
  ch2_pulse = pulseIn(ch2, HIGH);
  ch3_pulse = pulseIn(ch3, HIGH);
  ch4_pulse = pulseIn(ch4, HIGH);

  ch1_power = ((float)ch1_pulse/500.0f) - 3;
  ch2_power = ((float)ch2_pulse/500.0f) - 3;
  ch3_power = (-(float)ch3_pulse/500.0f) + 3;
  ch4_power = ((float)ch4_pulse/500.0f) - 3;

  if (abs(ch1_power) < 0.05){
    ch1_power = 0;
  }
  if (abs(ch2_power) < 0.05){
    ch2_power = 0;
  }
  if (abs(ch3_power) < 0.05){
    ch3_power = 0;
  }
  if (abs(ch4_power) < 0.05){
    ch4_power = 0;
  }


    Serial.print(ch1_power);
    Serial.print(":");
    Serial.print(ch2_power);
    Serial.print(":");
    Serial.print(ch3_power);
    Serial.print(":");
    Serial.println(ch4_power);

    m1_duty = ch3_power + ch4_power + ch1_power;
    m2_duty = ch3_power - ch4_power - ch1_power;
    m3_duty = ch3_power - ch4_power + ch1_power;
    m4_duty = ch3_power + ch4_power - ch1_power;

    int max_duty = findMax(m1_duty, m2_duty, m3_duty, m4_duty);

    if (max_duty > 1){
      m1_duty /= max_duty;
      m2_duty /= max_duty;
      m3_duty /= max_duty;
      m4_duty /= max_duty;
    }

    m1.setTargetDutyCycle(m1_duty);
    m2.setTargetDutyCycle(m2_duty);
    m3.setTargetDutyCycle(m3_duty);
    m4.setTargetDutyCycle(m4_duty);

    m1.runControlLoop();
    m2.runControlLoop();
    m3.runControlLoop();
    m4.runControlLoop();
}

int findMax(int a, int b, int c, int d) {
  int maxVal = a;

  if (b > maxVal) maxVal = b;
  if (c > maxVal) maxVal = c;
  if (d > maxVal) maxVal = d;

  return maxVal;
}


