/*************************************************************
Author: Hao Dong
Date: 2018.06.23
Email:  hao.dong.nanjing@gmail.com
*************************************************************/

#include "pid.h"

// kp and ki are the parameter generated by s transfer function
void Init_pidStruct(pidStruct * s, float kp, float ki, float ts) {
    s->kp = kp ;
    s->ki = ki * ts ;

    s->sum_ki = 0 ;
    s->output = 0 ;
    s->reference = 0 ;
    s->isum_high = 400.0 ;
    s->isum_low = -400.0;
}

float Calc_pidStruct(pidStruct * s, float error) {
    s->sum_ki += s->ki * error ;

    if(s->sum_ki > s->isum_high ){
        s->sum_ki = s->isum_high ;
    }
    else if( s->sum_ki < s->isum_low ){
        s->sum_ki = s->isum_low ;
    }
    s->output = s->sum_ki + s->kp * error ;
    return s->output ;
}
