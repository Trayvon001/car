


#include "include.h"
#include "common.h"
float sever_KP=10/1000.0 ;
float sever_KI=0.0/1000 ;
float sever_KD=1.0/1000;

int sever_PID(int error,float kp,float ki,float kd)
{
    static float Lastsever_Error = 0.0f;
    static float ErrorInter = 0.0f;
    const float  ErrorInterLimit = 2000.0f;
    static uint8_t FirstTimeFlag = TRUE;    // PD第一次标志
    float Error = error;
    int sever_duty_return;
    if(FirstTimeFlag)
    {
        sever_duty_return = kp * Error;
        FirstTimeFlag = FALSE;
    }
    else sever_duty_return = kp * Error + ki*ErrorInter + kd * (Error - Lastsever_Error);

    Lastsever_Error = Error;
    ErrorInter += Error;
    if(ErrorInter > ErrorInterLimit) 
        ErrorInter = ErrorInterLimit;
    else if (ErrorInter < -ErrorInterLimit)
        ErrorInter = -ErrorInterLimit;
    return sever_duty_return;
}
