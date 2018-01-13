#include "pid.h"

void PID::init(PID_PARAMETERS param){
    pid_param.e = param.e;
    pid_param.e_ = param.e_;
    pid_param.e__ = param.e__;
    pid_param.Kd = param.Kd;
    pid_param.Ki = param.Ki;
    pid_param.Kp = param.Kp;
    pid_param.PID_Saturation = param.PID_Saturation;
    pid_param.Ts = param.Ts;
    pid_param.u = param.u;
    pid_param.u_ = param.u_;
    dPart = 0;
    dPart_ = 0;
    intergral = 0.0;
}

void PID::pid_set_k_params(float Kp,float Ki, float Kd)
{
    pid_param.Kp = Kp;
    pid_param.Ki = Ki;
    pid_param.Kd = Kd;
}

float PID::pid_process(float error)
{
    pid_param.e__ = pid_param.e_;
    pid_param.e_ = pid_param.e;
    pid_param.e = error;
    pid_param.u_ = pid_param.u;

    intergral = pid_param.Ki * pid_param.Ts * pid_param.e;
    if (intergral > 5)
        intergral = 5;
    else if (intergral < -5)
        intergral = -5;

    dPart = (pid_param.Kd / pid_param.Ts) * (pid_param.e - (2 * pid_param.e_) + pid_param.e__);
    dPart = dPart_ + 0.005 * (dPart - dPart_);    // 50Hz LPF @400Hz  LPF: 33hz. 1/2pi*f
    pid_param.u = pid_param.u_ 
            + pid_param.Kp * (pid_param.e - pid_param.e_);
    if (pid_param.u > pid_param.PID_Saturation)
    {
        pid_param.u = pid_param.PID_Saturation;
    }
    else if (pid_param.u < (-pid_param.PID_Saturation))
    {
        pid_param.u = -pid_param.PID_Saturation;
    }   
    if (pid_param.u > pid_param.PID_Saturation)
    {
        pid_param.u = pid_param.PID_Saturation;
    }
    else if (pid_param.u < (-pid_param.PID_Saturation))
    {
        pid_param.u = -pid_param.PID_Saturation;
    }
    dPart_ = dPart;
    return pid_param.u;
}

// float PID::pid_process(float error)
// {
//     // pid_param.e__ = pid_param.e_;
//     pid_param.e_ = pid_param.e;
//     pid_param.e = error;
//     // pid_param.u_ = pid_param.u;

//     intergral = intergral + pid_param.Ts * pid_param.e;
//     if (intergral > 2)
//         intergral = 2;
//     else if (intergral < -2)
//         intergral = -2;
//     dPart = pid_param.Kd * (pid_param.e - pid_param.e_) / pid_param.Ts;
//     dPart = dPart_ + 0.07 * (dPart - dPart_);
//     pid_param.u = pid_param.Kp * (pid_param.e)
//             + pid_param.Ki * intergral
//             + dPart;
//     dPart_ = dPart;        
//     // if (pid_param.u > pid_param.PID_Saturation)
//     // {
//     //     pid_param.u = pid_param.PID_Saturation;
//     // }
//     // else if (pid_param.u < (-pid_param.PID_Saturation))
//     // {
//     //     pid_param.u = -pid_param.PID_Saturation;
//     // }

//     return pid_param.u;
// }

void PID::pid_reset()
{
    pid_param.e = 0;
    pid_param.e_ = 0;
    pid_param.e__ = 0;
    pid_param.u = 0;
    pid_param.u_ = 0;
    dPart = 0;
    dPart_ = 0;
}
