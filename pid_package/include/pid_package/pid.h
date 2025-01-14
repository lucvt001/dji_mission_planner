#ifndef _PID_H_
#define _PID_H_

using namespace std;

class PID
{
public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    // PID( float Kp, float Ki, float Kd, float dt, float max, float min );
    PID ( float Kp, float Ki, float Kd, float dt, float max, float min )
        : Kp_(Kp), Ki_(Ki), Kd_(Kd), dt_(dt), max_(max), min_(min), prev_error_(0.0), integral_(0.0),
        condition_integral_input_max_(1e10), condition_integral_input_min_(-1e10),
        integral_max_(max_), integral_min_(min_),
        pid_name_("PID"), verbose_mode_(false) {};

    // Returns the manipulated variable given a setpoint and current process value
    float calculate( float setpoint, float current );

    // Set a range for the input in which the integrator will be active. Outside of this range, the integral will be reset to zero.
    void set_conditional_integral_input_range( float min, float max );

    // Set a range for the integral. If the integral is outside of this range, it will be clamped to the min or max value.
    void set_integral_range( float min, float max );

    // Enable verbose mode to print Pout, Iout, Dout and output
    void enable_verbose_mode(bool is_enable, string pid_name);

    // Reset the integrator and the previous error
    void reset();

private:
    float Kp_;
    float Ki_;
    float Kd_;
    float dt_;
    float max_;
    float min_;
    float prev_error_;
    float integral_;
    float condition_integral_input_max_;
    float condition_integral_input_min_;
    float integral_max_;
    float integral_min_;

    string pid_name_;
    bool verbose_mode_;
};

#endif