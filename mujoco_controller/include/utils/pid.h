#pragma once

#include <Eigen/Dense>
#include <fstream>
#include "utils/common_math.h"

class PIDController{

    public:

        PIDController(int dim, double dt);
        
        // supposed to be 3d output dimension
        void setGains(double kp, double ki, double kd);
        Eigen::VectorXd compute(const Eigen::Ref<const Eigen::VectorXd> &x_d, const Eigen::Ref<const Eigen::VectorXd> &x);
        double compute(const double x_d, const double x);

    private:
        int dim_;
        double kp_, ki_, kd_, dt_;
        Eigen::VectorXd i_control_;
        Eigen::VectorXd prev_error_;

};