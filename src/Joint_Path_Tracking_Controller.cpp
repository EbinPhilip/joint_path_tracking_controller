#include "Joint_Path_Tracking_Controller.h"
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

using namespace jpt_controller;

bool JointPathTrackingController::init(hardware_interface::PosVelJointInterface *hw,
                                       ros::NodeHandle &nh)
{
    return true;
}

bool JointPathTrackingController::init(hardware_interface::PosVelJointInterface *hw,
                                       ros::NodeHandle &root_nh, ros::NodeHandle &nh)
{
    return true;
}

void JointPathTrackingController::update(const ros::Time &time,
                                         const ros::Duration &period)
{

}

PLUGINLIB_EXPORT_CLASS(jpt_controller::JointPathTrackingController, controller_interface::ControllerBase)