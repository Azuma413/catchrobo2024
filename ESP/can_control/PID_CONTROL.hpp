#ifndef PID_CONTROL_HPP
#define PID_CONTROL_HPP
#include <Arduino.h>
struct pid_param
{
    float Kp;
    float Ki;
    float Kd;
    float _dead_zone;
    float _max_value;
    pid_param(float P = 0, float I = 0, float D = 0, float dead_zone = 100, float max_value = 10000)
    {
        Kp = P;
        Ki = I;
        Kd = D;
        _dead_zone = dead_zone;
        _max_value = max_value;
    }
};

class PID_CONTROL
{
public:
    PID_CONTROL() {}
    PID_CONTROL(float P, float I, float D, float dead_zone = 0, float max_value = 0)
    {
        Kp = P;
        Ki = I;
        Kd = D;
        _dead_zone = dead_zone;
        _max_value = max_value;
        integral = 0.0;
        prevError = 0.0;
        last_contrl_time = 0;
    }
    PID_CONTROL(pid_param par)
    {
        Kp = par.Kp;
        Ki = par.Ki;
        Kd = par.Kd;
        _dead_zone = par._dead_zone;
        _max_value = par._max_value;
        integral = 0.0;
        prevError = 0.0;
        last_contrl_time = 0;
    }
    double control(double error)
    {
        if (abs(error) < _dead_zone)
        {
            error = 0;
        }
        double time_p = micros() - last_contrl_time;
        if (last_contrl_time == 0 || time_p < 0)
        {
            time_p = 1;
        }
        time_p = 0.000001*time_p;
        double proportional = Kp * error;        
        integral += (time_p * Ki * error);
        if(abs(integral) > _max_value){
            integral= integral>0?_max_value:-_max_value;
        }
        if(abs(proportional) > _max_value){
            integral=0;
        }
        double derivative = 0;
        if (time_p != 0)
            derivative = Kd * (error - prevError) / time_p;
        prevError = error;
        double output = proportional + integral + derivative;
        last_contrl_time = micros();
        if (abs(output) > _max_value)
        {
            return output > 0 ? _max_value : -1 * _max_value;
        }
        return output;
    }
    void reset()
    {
        integral = 0.0;
        prevError = 0.0;
    }
    void setPram(float P = 0, float I = 0, float D = 0, float dead_zone = 0, float max_value = 0)
    {
        if (P != 0)
        {
            Kp = P;
        }
        if (I != 0)
        {
            Ki = I;
        }
        if (D != 0)
        {
            Kd = D;
        }
        if (dead_zone != 0)
        {
            _dead_zone = dead_zone;
        }
        if (max_value != 0)
        {
            _max_value = max_value;
        }
        reset();
    }
    void setPram(pid_param pid)
    {
        setPram(pid.Kp, pid.Ki, pid.Kd, pid._dead_zone, pid._max_value);
    }
    pid_param getParam()
    {
        return pid_param(Kp, Ki, Kd, _dead_zone, _max_value);
    }
    double operator>>(double error)
    {
        return control(error);
    }
    double operator<<(double error)
    {
        return control(error);
    }
    double Kp;
    double Ki;
    double Kd;
    double _max_value;
    double _dead_zone;
    double setpoint;
    double integral;
    double prevError;
    double last_contrl_time = 0;
};

#endif
