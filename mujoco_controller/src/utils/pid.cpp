#include "utils/pid.h"

PIDController::PIDController(int dim, double dt):dim_(dim), dt_(dt)
{
    i_control_.resize(dim_);
    prev_error_.resize(dim_);

    i_control_.setZero();
    prev_error_.setZero();

}
    
void PIDController::setGains(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

Eigen::VectorXd PIDController::compute(const Eigen::Ref<const Eigen::VectorXd> &x_d,
                                       const Eigen::Ref<const Eigen::VectorXd> &x)
{
    Eigen::VectorXd e;
    Eigen::VectorXd p_control;
    Eigen::VectorXd d_control;
    Eigen::VectorXd y;

    e = x_d - x;

    p_control = e;    
    i_control_ += e * dt_;
    d_control = (e - prev_error_) / dt_;

    y = kp_*p_control + ki_*i_control_ + kd_ * d_control;

    prev_error_ = e;

    return y;
}

double PIDController::compute(const double x_d, const double x)
{
    double e;
    double p_control;
    double d_control;
    double y;

    e = x_d - x;

    p_control = e;    
    i_control_(0) += e * dt_;
    d_control = (e - prev_error_(0)) / dt_;

    y = kp_*p_control + ki_*i_control_(0) + kd_ * d_control;

    prev_error_(0) = e;

    return y;
}