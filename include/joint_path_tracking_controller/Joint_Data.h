#ifndef __JPT_JOINT_DATA_H__
#define __JPT_JOINT_DATA_H__

#include <hardware_interface/posvel_command_interface.h>

#include <string>

namespace jpt_controller
{

    struct JointData
    {
        JointData(std::string jointName, hardware_interface::PosVelJointHandle joint_handle,
                  double pos_min, double pos_max, double speed_default, double speed_max)
            : name(jointName), handle(joint_handle), min_pos(pos_min), max_pos(pos_max),
              default_speed(speed_default), max_speed(speed_max)
        {
        }

        JointData(const jpt_controller::JointData&) = default;

        std::string name;

        hardware_interface::PosVelJointHandle handle;

        double current_start_pos = 0;
        double current_end_pos = 0;

        double min_pos;
        double max_pos;

        double default_speed;
        double max_speed;
    };

}

#endif