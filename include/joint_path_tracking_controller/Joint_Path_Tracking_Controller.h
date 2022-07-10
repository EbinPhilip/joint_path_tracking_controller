#ifndef __JOINT_PATH_TRACKING_CONTROLLER_H__
#define __JOINT_PATH_TRACKING_CONTROLLER_H__

#include "Joint_Data.h"

#include <ros/ros.h>
#include <hardware_interface/posvel_command_interface.h>
#include <controller_interface/controller.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <map>
#include <string>
#include <mutex>

namespace jpt_controller
{

    class JointPathTrackingController
        : public controller_interface::Controller<hardware_interface::PosVelJointInterface>
    {
    public:
        JointPathTrackingController() = default;

        virtual bool init(hardware_interface::PosVelJointInterface *hw,
                          ros::NodeHandle &nh) override;
        virtual bool init(hardware_interface::PosVelJointInterface *hw,
                          ros::NodeHandle &root_nh, ros::NodeHandle &nh) override;

        virtual void update(const ros::Time &time, const ros::Duration &period) override;

        virtual void starting(const ros::Time& time) override;
        virtual void stopping(const ros::Time& time) override;

    protected:
        void _holdCurrentPosition();

        hardware_interface::PosVelJointInterface *hw_;

        double waypoint_tolerance_ratio_;
        double goal_tolerance_ratio_;
        double speed_backoff_ratio_;

        bool preempted_ = false;
        std::mutex preempt_mutex_;

        bool goal_reached_ = false;

        bool has_goal = false;
        trajectory_msgs::JointTrajectory trajectory_;
        std::mutex trajectory_mutex_;
        unsigned int idx_ = 0;
        double waypoint_completion_ratio_ = 0.0;

        std::map<std::string, JointData> joint_map_;
    };

}

#endif