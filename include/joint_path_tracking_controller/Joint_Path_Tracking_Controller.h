#ifndef __JOINT_PATH_TRACKING_CONTROLLER__
#define __JOINT_PATH_TRACKING_CONTROLLER__

#include <ros/ros.h>
#include <hardware_interface/posvel_command_interface.h>
#include <controller_interface/controller.h>

namespace jpt_controller
{

class JointPathTrackingController 
    : public controller_interface::Controller<hardware_interface::PosVelJointInterface>
{
public:
    virtual bool init(hardware_interface::PosVelJointInterface* hw,
        ros::NodeHandle& nh) override;
    virtual bool init(hardware_interface::PosVelJointInterface* hw,
        ros::NodeHandle& root_nh, ros::NodeHandle& nh) override;
    
    virtual void update(const ros::Time& time, const ros::Duration& period) override;
};

}

#endif