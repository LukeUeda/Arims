#ifndef _MOTOR
#define _MOTOR
#include <Arduino.h>

enum ControlMode {POSITION_CONTROL, VELOCITY_CONTROL, DUTY_CYCLE_CONTROL};

class Motor{
private:
    int ia_pin;
    int ib_pin;
    int ha_pin;
    int hb_pin;

    int pulseCount;
    int desiredPulses;
    bool counting;

    long startTime;
    long stopTime;

    float kp;
    float ki;
    float kd;

    int encoder_state;

    float position;
    int position_enc;
    float velocity;
    int encoder_ppr;

    float target_position;
    float target_velocity;
    float target_duty_cycle;
    ControlMode control_mode;

    void setDutyCycle();
public:
    Motor(int ia, int ib, int ha, int hb);

    float duty_cycle;
    bool reverse; 

    float getVelocity();
    float getPosition();
    float getEncoderPosition();
    void logEncoderPulse();

    void setPID(int kp, int ki, int kd);
    void setEncoderPPR(int ppr);
    void setControlMode(ControlMode mode);

    void setTargetVelocity(float vel);
    void setTargetPosition(float pos);
    void setTargetDutyCycle(float d_cycle);

    void resetPosition();
    void runControlLoop();
};

#endif