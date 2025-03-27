#pragma once

#include <fstream>
#include <Eigen/Dense>
#include <chrono>


#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-collection.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/aba.hpp>


#include "utils/mob.h"

class Fr3ModelUpdater{
    
    public:
        static constexpr int kDof=7;

        Fr3ModelUpdater(){};    

        void initialize(const std::string urdf_path);
        // void getRobotState(); // get robot state information from Mujoco (ex: joint, angular velocity, ee pose ...)
        void updateModel(const Eigen::Ref<const Eigen::VectorXd> &q,
                         const Eigen::Ref<const Eigen::VectorXd> &qd,
                         const Eigen::Ref<const Eigen::VectorXd> &gw,
                         const Eigen::Ref<const Eigen::VectorXd> &tau); // update robot dynimics base on the current observation
        void updateKinematics();
        void updateDynamics();

        // void setTorque(const Eigen::Matrix<double, 7, 1> &torque_command);
        // void setPosition(const Eigen::Matrix<double, 7, 1> &position_command, bool idle_control = false);
        void setInitialValues();
        void setTimeStamp(const double t);
        // void setInitialValues(const Eigen::Isometry3d &transform);
        // void setInitialValues(const Eigen::Ref<const Eigen::Ref<const Eigen::VectorXd>> &q);
        // void printState();


    // private:

        pinocchio::Model model_;
        pinocchio::Data data_;

        MomentumObserver mob_{7, 80, 0.001};
        

        std::string base_frame_;
        std::string ee_frame_;
        int ee_frame_id_;        
        // std::string urdf_path_;
        
        // fr3 parameters --
        Eigen::Matrix<double, 7, 7> M_; // joint mass_matrix_;
        Eigen::Matrix<double, 7, 7> M_inv_; // inverse of joint mass_matrix_;
        Eigen::Matrix<double, 6, 6> A_; // lambda_matrix_;
        Eigen::Matrix<double, 7, 1> NLE_; // non-linear effect (corriolis + gravity)
        Eigen::Matrix<double, 7, 7> C_; // coriolis_;
        Eigen::Matrix<double, 7, 1> G_; // gravity_;

        Eigen::Matrix<double, 7, 1> q_init_; //initial joint angle; 
        Eigen::Matrix<double, 7, 1> q_; // current joint angle
        Eigen::Matrix<double, 7, 1> qd_; // current angular velocity
        Eigen::Matrix<double, 7, 1> qdd_; // current angluar acceleration

        Eigen::Matrix<double, 7, 1> tau_measured_; // from torque sensor
        Eigen::Matrix<double, 7, 1> tau_d_; // command torque
        Eigen::Matrix<double, 7, 1> tau_ext_;

        Eigen::Matrix<double, 6, 1> f_ext_;
        Eigen::Matrix<double, 6, 1> f_ee_ext_; // external force w.r.t end-effector frame
        Eigen::Matrix<double, 6, 1> f_measured_;

        Eigen::Matrix<double, 6, 7> J_;     // jacobian
        Eigen::Matrix<double, 7, 6> J_bar_; // dynamically consistent inverse

        Eigen::Matrix<double, 7, 7> I_ = Eigen::Matrix<double, 7,7>::Identity();
        Eigen::Matrix<double, 7, 7> N_; /// null space projector

        Eigen::Isometry3d initial_transform_;   ///< initial transform for idle control
        Eigen::Isometry3d transform_;

        Eigen::Matrix<double, 3, 1> p_; // end-effector position
        Eigen::Matrix<double, 3, 3> r_; // roation matrix of end-effector

        Eigen::Matrix<double, 6, 1> xd_; // end-effector velocity
        Eigen::Matrix<double, 6, 1> xd_prev_;
        Eigen::Matrix<double, 6, 1> xd_lpf_;

        Eigen::Matrix<double, 2, 1> gw_init_; // gripper initial width
        Eigen::Matrix<double, 2, 1> gw_; // griiper width
        
        double t_stamp_; // time stamp to set initial time of a given task

        // Eigen::Matrix<double, 7, 1> q_limit_center_;

        double delta_tau_max_{0.05};


        //   bool idle_controlled_ {false}; ///< it indicates that this arm is under the idle control status. that is FrankaModelUpdater has valid initial transform and is using the transform.
        //   bool target_updated_ {false}; ///< it is used to check whether any other action is on going excep the idle control
        // -- arm parameters

        //   ros::Time task_start_time_; ///< time when a task starts
        //   ros::Time task_end_time_; ///< you may use this to indicate the timeout

        //   static const double PRINT_RATE;
        //   int print_count_ {1000};
        // const ros::Duration print_rate_;
        // ros::Time next_print_time_;

        //   std::array<double, 16> F_T_EE_, EE_T_K_;

        //   std::ofstream q_out_file_ ;
        //   std::ofstream x_out_file_ ;

        //   std::string arm_name_;
};

    
