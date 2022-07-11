#ifndef __JPT_JOINT_DATA_H__
#define __JPT_JOINT_DATA_H__

#include <hardware_interface/posvel_command_interface.h>

#include <string>

namespace jpt_controller
{

    struct JointData
    {
        JointData(std::string jointName, hardware_interface::PosVelJointHandle joint_handle,
                  double pos_min, double pos_max, double speed_min, double speed_default,
                  double speed_max, double tolerance_waypoint, double tolerance_goal)
            : name(jointName), handle(joint_handle), min_pos(pos_min), max_pos(pos_max),
              min_speed(speed_min), default_speed(speed_default), max_speed(speed_max),
              waypoint_tolerance(tolerance_waypoint), goal_tolerance(tolerance_goal)
        {
        }

        JointData(const jpt_controller::JointData&) = default;

        std::string name;

        hardware_interface::PosVelJointHandle handle;

        // joint is enabled only if present in current trajectory(goal),
        // if disabled, must hold current position fetched from handle
        double enabled = false; 
        double current_wp_end_pos = 0;
        double current_pos;

        double min_pos;
        double max_pos;

        double min_speed;
        double default_speed;
        double max_speed;

        double waypoint_tolerance;
        double goal_tolerance;
    };

}

#endif