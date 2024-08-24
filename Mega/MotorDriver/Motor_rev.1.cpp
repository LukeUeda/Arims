#include "Motor.hpp"
#include <Arduino.h>

#define RESOLUTION_INDEX 11
#define RESOLUTION 2048

Motor::Motor(int ia, int ib, int ha, int hb){
    ia_pin = ia;
    ib_pin = ib;
    ha_pin = ha;
    hb_pin = hb;

    control_mode = VELOCITY_CONTROL;

    pinMode(ia_pin, OUTPUT);
    pinMode(ib_pin, OUTPUT);

    pinMode(ha_pin, INPUT);
    pinMode(hb_pin, INPUT);

    analogWriteResolution(RESOLUTION_INDEX); //Gives 2^RESOLUTION_INDEX resolution (RESOLUTION)
}




float Motor::getVelocity(){

    if (pulseCount == 0 && !counting) {
        startTime = millis(); // Start counting time at the first pulse
        counting = true;
    }

    pulseCount++; // Increment pulse count

    if (pulseCount >= desiredPulses) {
        stopTime = millis();
        unsigned long duration = stopTime - startTime; // Calculate duration

        // Reset variables for the next measurement
        pulseCount = 0;
        counting = false;

        velocity = 60000/duration; // resultant rpm

        Serial.print("Velocity: ");
        Serial.print(velocity);
        Serial.println(" rpm");
        return velocity;
    }
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

void Motor::configEncoder(int ppr, int HS_A){
    encoder_ppr = ppr; // input pulses per rotation
    encoderA_pin = HS_A; // input interrupt pin encoderA pin is connected to

    pinMode(encoderA_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderA_pin), getVelocity, RISING);
}

void Motor::setControlMode(ControlMode mode){
    control_mode = mode;
}



void Motor::setTargetVelocity(float vel){
    target_velocity = vel; //max 251rpm





}



void Motor::setTargetPosition(float pos){
    target_position = pos;
}

void Motor::setTargetDutyCycle(float duty_cycle){
    target_duty_cycle = duty_cycle;
}

void Motor::resetPosition(){
    position_enc = 0;
    position = 0;
}

void Motor::runControlLoop(){
    switch (control_mode)
    {
    case VELOCITY_CONTROL:
        break;
    
    case POSITION_CONTROL:
        break;

    case DUTY_CYCLE_CONTROL:
        duty_cycle = target_duty_cycle
        setDutyCycle();
        break;
    }
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
