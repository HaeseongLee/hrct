#include "utils/motion_primitives.h"

namespace primitives
{
    Eigen::Isometry3d approach(const double vel, // target velocity
                               const double t,   // current time
                               const double t_0, // initial time
                               const int dir)    // motion direction, default : z-axis
    {

        // Purpose
        //     Generate a linear otion of an end-effector toward an object
        // Output
        //     desired pose (6d) w.r.t end-effector. It should be transformed into a command w.r.t the robot base.

        Eigen::Isometry3d p;
        Eigen::Vector2d x;
        double v, d;
        double t_buffer = 0.2; // to avoid step input

        p = Eigen::Isometry3d::Identity();

        x = common_math::trapezoid(t, t_0, t_buffer, vel);
        d = x(0);
        v = x(1);

        p.translation()(dir) = d;

        return p;
    }

    // Eigen::Isometry3d spiral_motion(const double pitch,
    //                                 const double lin_vel,
    //                                 const double t,
    //                                 const double t_0,
    //                                 const double duration,
    //                                 double direction)
    // // Generate spiral trajectory w.r.t the end-effector frame.
    // // The end-effector will move on the X-Y plane keeping its initial orientation.
    // // Also, no movement along Z-axis.
    // {
    //     Eigen::Isometry3d x;
    //     Eigen::Vector2d traj;

    //     x = Eigen::Isometry3d::Identity();

    //     traj = dyros_math::spiral(t, t_0, t_0 + duration, Eigen::Vector2d{0, 0}, lin_vel, pitch, direction);
    //     x.translation().head<2>() = traj;

    //     return x;
    // }

}