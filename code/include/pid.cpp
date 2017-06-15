#include "pid.h"

Pid::Pid(float kp, float ki, float kd){
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _sum_errors = 0;
    _previous_error = 0;
    _correction = 0;
}

void Pid::setKp(float kp){
    _kp = kp;
}

void Pid::setKi(float ki){
    _ki = ki;
}

void Pid::setKd(float kd){
    _kd = kd;
}


int16_t Pid::update(int16_t target, int16_t state){
    float error = target - state ;
    _sum_errors += error;
    _correction += _kp * error + _ki * _sum_errors + _kd * (error - _previous_error);
    _previous_error = error;
    return (int16_t) _correction;
}

void Pid::reset(){
    _sum_errors = 0;
    _previous_error = 0;
    _correction = 0;
}
