#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm>

class PIDController {
public:
    PIDController(double kp, double ki, double kd,
                  double setpoint = 0.0,
                  double min_limit = -100.0,
                  double max_limit = 100.0)
        : kp_(kp), ki_(ki), kd_(kd),
          setpoint_(setpoint),
          min_limit_(min_limit),
          max_limit_(max_limit),
          prev_error_(0.0),
          integral_(0.0) {}
    
    double setpoint_;

    double update(double current_value, double dt) {
        double error = setpoint_ - current_value;

        // P term
        double p_term = kp_ * error;

        // I term with anti-windup
        integral_ += error * dt;

        if (ki_ != 0.0) {
            double i_max = (max_limit_ - p_term) / ki_;
            double i_min = (min_limit_ - p_term) / ki_;
            integral_ = std::clamp(integral_, i_min, i_max);
        }

        double i_term = ki_ * integral_;

        // D term
        double derivative = (error - prev_error_) / dt;
        double d_term = kd_ * derivative;

        double output = p_term + i_term + d_term;
        output = std::clamp(output, min_limit_, max_limit_);

        prev_error_ = error;
        return output;
    }

    void set_setpoint(double setpoint) { setpoint_ = setpoint; }

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }

private:
    double kp_, ki_, kd_;

    double min_limit_, max_limit_;
    double prev_error_;
    double integral_;
};

#endif
