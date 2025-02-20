#pragma once

#include <Eigen/Dense>
#include <fstream>
#include "utils/common_math.h"

class MomentumObserver{

    public:

        MomentumObserver(int dof, double k, double dt);
        
        //
        Eigen::VectorXd run(const Eigen::Ref<const Eigen::MatrixXd> &M,
                            const Eigen::Ref<const Eigen::MatrixXd> &C,
                            const Eigen::Ref<const Eigen::VectorXd> &G,
                            const Eigen::Ref<const Eigen::VectorXd> &tau,
                            const Eigen::Ref<const Eigen::VectorXd> &qd);

    private:
        double k_, dt_;
        Eigen::VectorXd p_; // momentum
        Eigen::VectorXd r_; // residual
        Eigen::VectorXd beta_; // G - C'qd
        Eigen::VectorXd tau_, tau_prev_;
        Eigen::VectorXd integral_;
        
        int debug_cnt_;

        
};