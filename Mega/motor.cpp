#include "Motor.hpp"
#include <Arduino.h>

#define RESOLUTION_INDEX 8
#define RESOLUTION 255

Motor::Motor(int ia, int ib, int ha, int hb){
    ia_pin = ia;
    ib_pin = ib;
    ha_pin = ha;
    hb_pin = hb;

    control_mode = VELOCITY_CONTROL;

    pulseCount = 0;

    duty_cycle = 0;

    pinMode(ia_pin, OUTPUT);
    pinMode(ib_pin, OUTPUT);

    pinMode(ha_pin, INPUT);
    pinMode(hb_pin, INPUT);
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

void Motor::logEncoderPulse(){
  position_enc += 1;
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
    target_velocity = vel; //max 251rpm
}


void Motor::setTargetPosition(float pos){
    target_position = pos;
}

void Motor::setTargetDutyCycle(float d_cycle){
    target_duty_cycle = d_cycle;
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
        if(reverse){
          duty_cycle = -target_duty_cycle;
        }else{
          duty_cycle = target_duty_cycle;
        }
        setDutyCycle();
        break;
    }
   }

void Motor::setDutyCycle(){
    if(duty_cycle > 0){
        analogWrite(ia_pin, 0);
        analogWrite(ib_pin, duty_cycle * RESOLUTION);
    } else {
        analogWrite(ia_pin, duty_cycle * -1 *  RESOLUTION);
        analogWrite(ib_pin, 0);
    }
}
