#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid_package/pid.h"

using namespace std;

float PID::calculate( float setpoint, float current )
{
    
    // Calculate error
    float error = setpoint - current;

    // Proportional term
    float Pout = Kp_ * error;

    // Integral term
    float Iout = 0.0;
    // If the error changes sign, instantly unwind the integral
    // Also unwind the integral if the input is outside the conditional range
    if ( integral_ * error < -1e-8 || current < condition_integral_input_min_ || current > condition_integral_input_max_) 
        integral_ = 0.0;
    // Only accumulate the integral term if the input is within the conditional range
    else
    {
        integral_ += error * dt_;
        Iout = Ki_ * integral_;
    } 
    
    // Clamp Iout
    if ( Iout > integral_max_ )
        Iout = integral_max_;
    else if ( Iout < integral_min_ )
        Iout = integral_min_;
    integral_ = Iout / Ki_; // Clamp back the integral term

    // Derivative term
    float derivative = (error - prev_error_) / dt_;
    float Dout = Kd_ * derivative;

    // Calculate total output
    float output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > max_ )
        output = max_;
    else if( output < min_ )
        output = min_;

    // Save error to previous error
    prev_error_ = error;

    if ( verbose_mode_ )
        cout << pid_name_ << " - proportional: " << Pout << " integral: " << Iout << " derivative: " << Dout << " Output: " << output << endl;

    return output;
}

void PID::set_conditional_integral_input_range( float min, float max )
{
    condition_integral_input_min_ = min;
    condition_integral_input_max_ = max;
}

void PID::set_integral_range( float min, float max )
{
    integral_min_ = min;
    integral_max_ = max;
}

void PID::reset()
{
    prev_error_ = 0.0;
    integral_ = 0.0;
}

void PID::enable_verbose_mode(bool is_enable, string pid_name)
{
    verbose_mode_ = is_enable;
    pid_name_ = pid_name;
}

#endif