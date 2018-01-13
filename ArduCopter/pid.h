#ifndef PID_H
#define PID_H
#pragma once

class PID {
public:
    typedef struct
    {
        float Kp;
        float Ki;
        float Kd;
        float Ts;
        float PID_Saturation;
        float e;
        float e_;
        float e__;
        float u;
        float u_;
    } PID_PARAMETERS;
    float pid_lpf_value;
    
    void init(PID_PARAMETERS param);
    float pid_process(float error);
    void pid_reset();
    void pid_set_k_params(float Kp,float Ki, float Kd);

private:
   PID_PARAMETERS pid_param;
   float intergral;
   float dPart;
   float dPart_;
};

#endif // PID_H
