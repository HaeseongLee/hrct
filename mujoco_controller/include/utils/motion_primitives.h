#pragma once

#include <Eigen/Dense>
#include <iostream>
#include "utils/common_math.h"
#include <math.h>

namespace primitives
{
    Eigen::Isometry3d approach(const double vel,
                               const double t,
                               const double t_0,
                               const int dir = 2); // 0 : x-axis, 1 : y-axis, 2 : z-axis

    // Eigen::Isometry3d spiral_motion(const double pitch,
    //                                 const double lin_vel,
    //                                 const double t,
    //                                 const double t_0,
    //                                 const double duration,
    //                                 double direction = 1); // 1 = cw, -1 = ccw

    // Eigen::Vector2d trapezoid(const double t,
    //                           const double t_0,
    //                           const double v_sat);

    // double push(const double force,
    //             const double t,
    //             const double t_0,
    //             const double duration);
                
    // Eigen::Isometry3d hold_pose();

    // void accum_tau(Eigen::Vector7d &accum_tau,
    //                const Eigen::Ref<const Eigen::Vector7d> &tau_ext);

    // void accum_wrench(Eigen::Vector6d &accum_wrench,
    //                   const Eigen::Ref<const Eigen::Vector6d> &f_ext);

    // Eigen::Vector3d wrtEndEffector(const Eigen::Isometry3d &world,
    //                                const Eigen::Vector3d &x,
    //                                bool debug);

    // Eigen::Vector3d wrtBaseFrame (const Eigen::Isometry3d &local,
    //                               const Eigen::Vector3d &x);

    // bool detectHole(const Eigen::Vector3d &f_ext,
    //                 const double threshold);     

    // Eigen::Matrix3d wiggle_motion(const double angle,
    //                               const double w, // rad/s
    //                               const double t,
    //                               const double t_0);

    // Eigen::Isometry3d tilt_motion(const Eigen::Vector3d cor, // center of rotation, (tcp)
    //                               const Eigen::Vector3d axis,
    //                               const double angle);

    // Eigen::Matrix3d yawing_motion(const double angle,
    //                               const double w, // rad/s
    //                               const double t,
    //                               const double t_0);

    // Eigen::MatrixXd pouring_motion(double pouring_angle,                          //Pouring angle [deg]
    //                                 const Eigen::Vector3d& pouring_axis,			//Pouring axis w.r.t {Base}
    //                                 const Eigen::Vector3d& pouring_radius,		    //Pouring radius w.r.t {Base}
    //                                 const Eigen::Vector3d& pouring_translation,	    //Translation of rotation axis during pouring w.r.t {Base}
    //                                 const Eigen::Vector3d& pos_0,                   //Initial EE position
    //                                 const Eigen::Matrix3d& rot_0,                   //Initial EE rotation
    //                                 const double t,
    //                                 const double t_0,
    //                                 double duration,
    //                                 bool return_dot);
}