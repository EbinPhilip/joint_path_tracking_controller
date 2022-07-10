#include "Joint_Path_Tracking_Controller.h"
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

#include <xmlrpcpp/XmlRpcValue.h>

using namespace jpt_controller;
using namespace XmlRpc;

bool JointPathTrackingController::init(hardware_interface::PosVelJointInterface *hw,
                                       ros::NodeHandle &nh)
{
    assert(hw);
    hw_ = hw;

    XmlRpcValue joints;
    if (!nh.getParam("waypoint_tolerance_ratio", waypoint_tolerance_ratio_) || !nh.getParam("goal_tolerance_ratio", goal_tolerance_ratio_) || !nh.getParam("speed_backoff_ratio", speed_backoff_ratio_) || !nh.getParam("joints", joints))
    {
        ROS_ERROR("JointPathTrackingController: parameters missing");
        throw std::runtime_error("JointPathTrackingController: parameters missing");
    }

    if (joints.getType() != XmlRpcValue::TypeArray)
    {
        ROS_ERROR("JointPathTrackingController: expected array type!");
        throw std::runtime_error("JointPathTrackingController: expected array type!");
    }

    for (int i = 0; i < joints.size(); ++i)
    {
        if (joints[i].getType() != XmlRpcValue::TypeStruct)
        {
            ROS_ERROR("JointPathTrackingController: expected map type!");
            throw std::runtime_error("JointPathTrackingController: expected map type!");
        }

        std::string name = joints[i]["name"];
        double min_pos = joints[i]["min_pos"];
        double max_pos = joints[i]["max_pos"];

        double default_speed = joints[i]["default_speed"];
        double max_speed = joints[i]["max_speed"];

        auto handle = hw_->getHandle(name);

        JointData joint = JointData(name, handle, min_pos, max_pos, default_speed, max_speed);
        joint_map_.insert(std::make_pair(name, joint));
    }

    return true;
}

bool JointPathTrackingController::init(hardware_interface::PosVelJointInterface *hw,
                                       ros::NodeHandle &root_nh, ros::NodeHandle &nh)
{
    return init(hw, nh);
}

void JointPathTrackingController::update(const ros::Time &time,
                                         const ros::Duration &period)
{
    if (!preempted_ && has_goal)
    {
        std::lock_guard<std::mutex> trajectory_lock(trajectory_mutex_);

        if (!goal_reached_)
        {
            // if current waypoint has been reached, load the next one
            if (waypoint_completion_ratio_ >= waypoint_tolerance_ratio_)
            {
                // not the last waypoint in the current path
                if (idx_ < trajectory_.points.size() - 1)
                {
                    ++idx_;
                    for (int i = 0; i < trajectory_.joint_names.size(); ++i)
                    {
                        auto joint = joint_map_.at(trajectory_.joint_names[i].c_str());
                        joint.current_start_pos = joint.handle.getPosition();
                        joint.current_end_pos = trajectory_.points[idx_].positions[i];
                    }
                }
                else // last waypoint/goal
                {
                    if (waypoint_completion_ratio_ >= goal_tolerance_ratio_)
                    {
                        goal_reached_ = true;
                    }
                }
            }

            // find joint with largest normalized position difference
            std::string max_distance_joint = "";
            double max_distance = -1.0;
            for (auto &joint_pair : joint_map_)
            {
                auto &joint = joint_pair.second;

                double distance = (joint.current_end_pos - joint.handle.getPosition()) 
                        / (joint.max_pos - joint.min_pos); // normalize position difference
                if (distance > max_distance)
                {
                    max_distance = distance;
                    max_distance_joint = joint_pair.first;
                }
            }

            // get joint which has the largest normalized position difference("max joint")
            JointData& max_joint = joint_map_.at(max_distance_joint);
            // get max scaled speed for the "max joint"               
            double max_speed = max_joint.max_speed;
            // use position difference and max_speed to get the time required to reach waypoint
            double wp_time = (max_joint.current_end_pos - max_joint.handle.getPosition())/max_speed;

            // calculate speed for each joint using common wp_time
            // but individual postion differences
            for (auto& joint_pair : joint_map_)
            {
                auto& joint = joint_pair.second;
                if (joint_pair.first == max_distance_joint) // skip "max joint"
                {
                    joint.handle.setCommand(joint.current_end_pos, max_speed);
                    continue;
                }

                double speed = (joint.current_end_pos-joint.handle.getPosition()) / wp_time;

                joint.handle.setCommand(joint.current_end_pos, std::min(speed, joint.max_speed));
            }
        }
        else // goal reached
        {
            // hold the goal position
            for (auto& joint_pair : joint_map_)
            {
                auto& joint = joint_pair.second;
                joint.handle.setCommand(joint.current_end_pos, joint.default_speed);
            }
        }
    }
    else // preempted or no goal provided
    {
        _holdCurrentPosition();
    }
}
void JointPathTrackingController::starting(const ros::Time &time)
{
    std::lock_guard<std::mutex> trajectory_lock(trajectory_mutex_);
    if (isRunning())
    {
        return;
    }
    idx_ = 0;
    trajectory_ = trajectory_msgs::JointTrajectory();
    waypoint_completion_ratio_ = 0.0;
}

void JointPathTrackingController::stopping(const ros::Time &time)
{
    std::lock_guard<std::mutex> preempt_lock(preempt_mutex_);
    preempted_ = true;

    if (this->isStopped())
    {
        return;
    }

    std::lock_guard<std::mutex> trajectory_lock(trajectory_mutex_);
    idx_ = 0;
    trajectory_ = trajectory_msgs::JointTrajectory();
    waypoint_completion_ratio_ = 0.0;
}

// stop and hold at current position
void JointPathTrackingController::_holdCurrentPosition()
{
    for (auto &joint : joint_map_)
    {
        auto &handle = joint.second.handle;
        handle.setCommand(handle.getPosition(), joint.second.default_speed);
    }
}

PLUGINLIB_EXPORT_CLASS(jpt_controller::JointPathTrackingController, controller_interface::ControllerBase)