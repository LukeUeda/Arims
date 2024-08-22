#include "Motor.hpp"
#include <Arduino.h>

#define RESOLUTION_INDEX 11
#define RESOLUTION 2048

Motor::Motor(int ia, int ib, int ha, int hb){
    ia_pin = ia;
    ib_pin = ib;
    ha_pin = ha;
    hb_pin = hb;

    pinMode(ia_pin, OUTPUT);
    pinMode(ib_pin, OUTPUT);

    pinMode(ha_pin, INPUT);
    pinMode(hb_pin, INPUT);

    analogWriteResolution(RESOLUTION_INDEX); //Gives 2^RESOLUTION_INDEX resolution (RESOLUTION)
}

float Motor::getVelocity(){
    return velocity;
}

float Motor::getPosition(){
    return position;
}

float Motor::getEncoderPosition(){
    return position_enc;
}

void Motor::setPID(int _kp, int _ki, int _kd){
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

void Motor::setEncoderPPR(int ppr){
    encoder_ppr = ppr;
}

void Motor::setControlMode(ControlMode mode){
    control_mode = mode;
}

void Motor::setTargetVelocity(float vel){
    target_velocity = vel;
}

void Motor::setTargetPosition(float pos){
    target_position = pos;
}

void Motor::resetPosition(){
    position_enc = 0;
    position = 0;
}

void Motor::runControlLoop(){

}

void Motor::setDutyCycle(){
    if(duty_cycle < 1){
        analogueWrite(ia_pin, 0);
        analogueWrite(ib_pin, duty_cycle * -1 * RESOLUTION);
    } else {
        analogueWrite(ia_pin, duty_cycle * RESOLUTION);
        analogueWrite(ib_pin, 0);
    }
}
