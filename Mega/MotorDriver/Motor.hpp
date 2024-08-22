#ifndef _MOTOR
#define _MOTOR

enum ControlMode {POSITION_CONTROL, VELOCITY_CONTROL};

class Motor{
private:
    int ia_pin;
    int ib_pin;
    int ha_pin;
    int hb_pin;

    float kp;
    float ki;
    float kd;

    float duty_cycle;
    float position;
    int position_enc;
    float velocity;
    bool reverse;
    int encoder_ppr;

    float target_position;
    float target_velocity;
    ControlMode control_mode;

    void setDutyCycle();
public:
    Motor(int ia, int ib, int ha, int hb);

    float getVelocity();
    float getPosition();
    float getEncoderPosition();

    void setPID(int kp, int ki, int kd);
    void setEncoderPPR(int ppr);
    void setControlMode(ControlMode mode);

    void setTargetVelocity(float vel);
    void setTargetPosition(float pos);

    void resetPosition();
    void runControlLoop();
};

#endif