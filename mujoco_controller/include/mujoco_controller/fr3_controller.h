#pragma once

#include <Eigen/Dense>
#include <fstream>

#include "robot_model/fr3_updater.h"
#include "utils/common_math.h"
#include "utils/time_scheduler.h"
#include "utils/motion_primitives.h"

using namespace common_math;
using namespace primitives;

namespace MjController{
    
    class Fr3Controller{
        public:
            Fr3Controller(const std::string urdf_path);

            void initialize(const std::string urdf_path);
            // void getRobotState(); // from python code

            void updateModel(const Eigen::Ref<const Eigen::VectorXd> &q,
                             const Eigen::Ref<const Eigen::VectorXd> &qd,
                             const Eigen::Ref<const Eigen::VectorXd> &tau,
                             const Eigen::Ref<const Eigen::VectorXd> &ft,
                             const double t); // update kinematics and dynamics

            Eigen::VectorXd setIdleConfig(const Eigen::Ref<const Eigen::Vector2d> &time);
            Eigen::VectorXd gripperOpen(const double target_width, const Eigen::Ref<const Eigen::Vector2d> &time);
            Eigen::VectorXd gripperClose();

            Eigen::VectorXd taskMove(const Eigen::Ref<const Eigen::Vector3d> &x_goal, // goal position
                                     const Eigen::Ref<const Eigen::MatrixXd> &r_goal, // goal rotation
                                     const Eigen::Ref<const Eigen::Vector2d> &time);  // time information [t_0, t_f]

            Eigen::VectorXd pick(const Eigen::Ref<const Eigen::VectorXd> &x_goal, // goal position
                                 const Eigen::Ref<const Eigen::MatrixXd> &r_goal, // goal rotation
                                 const double duration);

            Eigen::VectorXd place(const Eigen::Ref<const Eigen::VectorXd> &x_goal, // goal position
                                  const Eigen::Ref<const Eigen::MatrixXd> &r_goal, // goal rotation
                                  const double duration);

            Eigen::VectorXd pickAndPlace(const Eigen::Ref<const Eigen::MatrixXd> &x_goals,
                                         const Eigen::Ref<const Eigen::MatrixXd> &r_goals,
                                         const Eigen::Ref<const Eigen::VectorXd> &durations);

            Eigen::VectorXd approach(const double v, const double t_0, const int dir=2);
            // Eigen::VectorXd search();
            // Eigen::VectorXd insert();
            Eigen::VectorXd masi(const Eigen::Ref<const Eigen::VectorXd> &x_goal, // goal position
                                 const Eigen::Ref<const Eigen::MatrixXd> &r_goal, // goal rotation
                                 const double duration);

            Eigen::VectorXd pegInHole(const Eigen::Ref<const Eigen::MatrixXd> &x_goals,
                                      const Eigen::Ref<const Eigen::MatrixXd> &r_goals,
                                      const Eigen::Ref<const Eigen::VectorXd> &durations);

            // whether the given task is completed or not     
            Eigen::VectorXd idleState();
            void idleStateWithGrasp();
            void saveState();

            bool next_task_;
            bool set_attach_;

        private:

            Fr3ModelUpdater model_;

            std::string urdf_path_;
            
            std::vector<std::string> joint_names_;

            Eigen::Vector7d q_idle_; // ready to start task
            Eigen::Vector7d q_init_; 
            Eigen::Vector7d q_desired_;
            Eigen::Vector7d q_, qd_; // current joint and velocity
            Eigen::Vector7d tau_d_; // robot torque command
            Eigen::Vector7d tau_measured_;

            Eigen::Vector6d f_ext_;
            Eigen::Vector6d f_bias_;

            Eigen::Vector2d gw_; //grasp width
            Eigen::Vector2d gw_init_;       
            Eigen::Vector2d gw_desired_;
            Eigen::Vector2d gv_; //grasp velocity 
            Eigen::Vector2d gf_; //grasp force

            Eigen::Vector8d ctrl_;
            double t_;
            double t_init_;

            int dof_ = 7;
            double hz_ = 1000.0;

            bool set_init_;

            std::vector<std::string> pick_task_ = {"reach", "align", "close", "lift"};
            Eigen::VectorXd pick_periods_;
            Eigen::VectorXd pick_timeline_;           
            TimeScheduler pick_sch_;

            std::vector<std::string> place_task_ = {"reach", "open", "homing"};
            Eigen::VectorXd place_periods_;
            Eigen::VectorXd place_timeline_;           
            TimeScheduler place_sch_;

            std::ofstream save_joint_q{"ros2_ws/src/mujoco_controller/joint_q.txt"};
            
    };

}
