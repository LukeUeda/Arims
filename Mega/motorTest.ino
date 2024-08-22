#include "MotorDriver/Motor.hpp"

int ia = 2;
int ib = 5;

Motor m1;

String command;

void setup(){
    m1 = Motor(ia, ib, 0, 0);
    m1.setControlMode(DUTY_CYCLE_CONTROL);
    Serial.begin(9600);
}

void loop(){
    if (Serial.available()) {
        command = Serial.readStringUntil('\n');
        m1.setTargetDutyCycle(float(command));

        Serial.println("Setting duty cycle to ")
    }
}