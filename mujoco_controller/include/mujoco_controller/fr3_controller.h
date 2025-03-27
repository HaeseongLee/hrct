#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <unordered_map>

#include "robot_model/fr3_updater.h"
#include "utils/common_math.h"
#include "utils/time_scheduler.h"
#include "utils/motion_primitives.h"
#include "utils/estimator.h"
#include "utils/pid.h"

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
            Eigen::VectorXd search(double p, double v, double f, double t, double t_0, double duration);
            Eigen::VectorXd insert(double f, double t, double t_0);

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
            Eigen::Vector7d tau_d_lpf_; // prevent large transition torque
            Eigen::Vector7d tau_measured_; // from joint torque sensor
            Eigen::Vector7d tau_ext_; // external torque (friction + contact)
            
            Eigen::Vector3d x_; // end_effector position w.r.t base
            Eigen::Vector3d x_ee_; // end_effector position w.r.t ee frame
            Eigen::Vector6d v_, v_lpf_; // end_effector velocity w.r.t base
            Eigen::Vector6d v_ee_, v_ee_lpf_;
            Eigen::Matrix3d R_; // end_effector rotation w.r.t base
            Eigen::Matrix3d R_ee_;
            Eigen::Vector6d f_ext_, f_ext_lpf_;
            Eigen::Vector6d f_contact_;
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

            PegInHoleEstimator estimator_ = PegInHoleEstimator(0.0005, 1.0, 3.0, 0.001);
            std::unordered_map<std::string, int> est_map_ = {
                {"move", 1}, {"approach", 2}, {"search", 3}, {"insert", 4}, {"homing", 5}};

            PIDController force_pid_ = PIDController(1, 0.001);
            PIDController insert_pid_ = PIDController(2, 0.001);


            std::ofstream save_joint_q{"ros2_ws/src/hrct/mujoco_controller/joint_q.txt"};
            std::ofstream save_joint_t{"ros2_ws/src/hrct/mujoco_controller/joint_t.txt"};
            std::ofstream save_task_p{"ros2_ws/src/hrct/mujoco_controller/task_p.txt"};
            std::ofstream save_task_v{"ros2_ws/src/hrct/mujoco_controller/task_v.txt"};
            std::ofstream save_f_ext{"ros2_ws/src/hrct/mujoco_controller/f_ext.txt"};
            std::ofstream save_t_ext{"ros2_ws/src/hrct/mujoco_controller/t_ext.txt"};
            std::ofstream debug_file{"ros2_ws/src/hrct/mujoco_controller/debug_file.txt"};


    };

}
