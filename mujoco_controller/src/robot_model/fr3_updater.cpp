#include "robot_model/fr3_updater.h"
#include <Eigen/Geometry>
#include <fstream>

// using namespace Eigen;

// Fr3ModelUpdater::Fr3ModelUpdater()
// {}

void Fr3ModelUpdater::initialize(const std::string urdf_path)
{   
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);

    // std::cout << "로봇의 전체 프레임 목록:" << std::endl;
    // for (size_t i = 0; i < model_.frames.size(); ++i) {
    //     std::cout << "Frame " << i << ": " << model_.frames[i].name << std::endl;
    // }
    base_frame_ = model_.frames[1].name;
    ee_frame_ = model_.frames.back().name;
    ee_frame_id_ = model_.getFrameId(ee_frame_); // suppose Jacobian for end-effector frame

    q_init_.setZero();
    gw_init_.setZero();
}

void Fr3ModelUpdater::updateModel(const Eigen::Ref<const Eigen::VectorXd> &q,
                                  const Eigen::Ref<const Eigen::VectorXd> &qd,
                                  const Eigen::Ref<const Eigen::VectorXd> &gw,
                                  const Eigen::Ref<const Eigen::VectorXd> &tau)
{
    q_ = q;
    qd_ = qd;
    gw_ = gw;
    tau_measured_ = tau;

    // compute core parameters
    pinocchio::crba(model_, data_, q_, pinocchio::Convention::WORLD); // compute Jabian & FK with Convention::WORLD
    // pinocchio::nonLinearEffects(model_, data_, q_, qd_);
    pinocchio::computeCoriolisMatrix(model_, data_, q_, qd_); // obtain corioli matrix for MOB
    pinocchio::computeGeneralizedGravity(model_, data_, q);   // obtain grative vector

    updateKinematics();
    updateDynamics();
}

void Fr3ModelUpdater::updateKinematics()
{
    // Get Jacobian
    pinocchio::getFrameJacobian(model_, data_, ee_frame_id_, pinocchio::WORLD, J_);
   
    // Task pose
    transform_ = data_.oMf[ee_frame_id_].toHomogeneousMatrix();
    p_ = transform_.translation();
    r_ = transform_.linear();

    xd_ = J_*qd_;
    // TODO : xd_prev_, xd_lfp_;    
    
}


void Fr3ModelUpdater::updateDynamics()
{
    M_ = data_.M;
    M_.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>(); // make full symetric matrix

    M_inv_ =  M_.inverse();

    C_ = data_.C;
    G_ = data_.g;

    // NLE_ = data_.nle; // C*q_dot + G
    NLE_ = C_*qd_ + G_;

    A_ = (J_*M_inv_*J_.transpose()).inverse(); // mass matrix in the task space, A = (J*M^-1*J^T)^-1
    J_bar_ = M_inv_*J_.transpose()*A_; // dynamically consistant inverse of jacobian        
    N_ = I_ - J_.transpose()*J_bar_.transpose(); // Null-space Projection

    tau_ext_ = mob_.run(M_, C_, G_, tau_measured_, qd_);

}
        
void Fr3ModelUpdater::setInitialValues()
{
    initial_transform_ = transform_;
    q_init_ = q_;
    gw_init_ = gw_;
    xd_.setZero(); 
}

void Fr3ModelUpdater::setTimeStamp(const double t)
{
    t_stamp_ = t;
}

// void Fr3ModelUpdater::setTorque(const Eigen::Matrix<double, 7, 1> &torque_command)
// {
//     tau_d_ = torque_command;
// }
