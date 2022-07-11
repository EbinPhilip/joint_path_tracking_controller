#ifndef __JOINT_PATH_TRACKING_CONTROLLER_H__
#define __JOINT_PATH_TRACKING_CONTROLLER_H__

#include "Joint_Data.h"

#include <ros/ros.h>
#include <hardware_interface/posvel_command_interface.h>
#include <controller_interface/controller.h>
#include <actionlib/server/simple_action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#include <map>
#include <string>
#include <mutex>
#include <memory>

namespace jpt_controller
{

    class JointPathTrackingController
        : public controller_interface::Controller<hardware_interface::PosVelJointInterface>
    {
    public:
        JointPathTrackingController();

        virtual bool init(hardware_interface::PosVelJointInterface *hw,
                          ros::NodeHandle &nh) override;
        virtual bool init(hardware_interface::PosVelJointInterface *hw,
                          ros::NodeHandle &root_nh, ros::NodeHandle &nh) override;

        virtual void update(const ros::Time &time, const ros::Duration &period) override;

        virtual void starting(const ros::Time& time) override;
        virtual void stopping(const ros::Time& time) override;
        void _executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

    protected:
        void _holdCurrentPosition();
        void _lockAndReplacePath(const trajectory_msgs::JointTrajectory& path);

        hardware_interface::PosVelJointInterface *hw_;

        typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> JointAction;
        std::unique_ptr<JointAction> as_;

        bool has_goal = false;

        // lock on this mutex must be acquired before writing to any of the following fields
        std::mutex preempt_mutex_;
        ////// start of fields synchronized by joint_path_mutex //////
        bool preempted_ = false;
        ////// end of fields synchronized by preempt_mutex_ //////


        // lock on this mutex must be acquired before writing to any of the following fields
        std::mutex joint_path_mutex_;
        ////// start of fields synchronized by joint_path_mutex //////
        unsigned int idx_ = 0; // path point index : 1 based
        trajectory_msgs::JointTrajectory path_;
        bool path_complete_ = false; // all path points have been achieved within waypoint tolerance
        bool waypoint_reached_ = true; // should be true at the start of each path
        bool goal_reached_ = false; // last path point has been achieved within goal tolerance
        std::map<std::string, JointData> joint_map_;
        ////// end of fields synchronized by joint_path_mutex //////
    };

}

#endif