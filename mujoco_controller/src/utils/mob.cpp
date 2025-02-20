#include "utils/mob.h"

MomentumObserver::MomentumObserver(int dof, double k, double dt):k_{k}, dt_(dt)
{
    p_.resize(dof);
    r_.resize(dof);
    beta_.resize(dof);
    tau_.resize(dof);
    tau_prev_.resize(dof);
    integral_.resize(dof);

    p_.setZero();
    r_.setZero();
    beta_.setZero();
    tau_.setZero();
    tau_prev_.setZero();
    integral_.setZero(); // assum p0 is a zero vector

    debug_cnt_ = 0;
};

Eigen::VectorXd MomentumObserver::run(const Eigen::Ref<const Eigen::MatrixXd> &M,
                                      const Eigen::Ref<const Eigen::MatrixXd> &C,
                                      const Eigen::Ref<const Eigen::VectorXd> &G,
                                      const Eigen::Ref<const Eigen::VectorXd> &tau,
                                      const Eigen::Ref<const Eigen::VectorXd> &qd)
{
    Eigen::VectorXd t_ext;

    p_ = M*qd;

    beta_ = -C.transpose()*qd + G;

    tau_ = tau + r_ - beta_; // integral{tau + C'qd - G + r}

    integral_ += 0.5*dt_*(tau_prev_ + tau_); // to reduce accumulation error

    r_ = k_*(p_ - integral_);

    t_ext = -r_;

    tau_prev_ = tau_;

    return t_ext;

}
