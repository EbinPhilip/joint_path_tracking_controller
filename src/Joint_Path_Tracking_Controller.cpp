#include "Joint_Path_Tracking_Controller.h"

#include <rotational_units/Rotational_Units.h>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

#include <control_msgs/FollowJointTrajectoryActionResult.h>

#include <xmlrpcpp/XmlRpcValue.h>
#include <functional>

using namespace jpt_controller;
using namespace XmlRpc;
using namespace RUnits;

JointPathTrackingController::JointPathTrackingController()
    : as_("FollowJointTrajectory", boost::bind(&JointPathTrackingController::_executeCB, this, _1), true)
{
}

bool JointPathTrackingController::init(hardware_interface::PosVelJointInterface *hw,
                                       ros::NodeHandle &nh)
{
    assert(hw);
    hw_ = hw;

    XmlRpcValue joints;
    if (!nh.getParam("joints", joints))
    {
        ROS_ERROR("JointPathTrackingController: parameter joint is missing");
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
        double min_pos, max_pos, min_speed, default_speed, max_speed;

        if (joints[i].hasMember("min_pos_deg"))
        {
            min_pos = ((Radians)Degrees(joints[i]["min_pos_deg"])).Value();
        } else {
            min_pos = joints[i]["min_pos"];
        }
        if (joints[i].hasMember("max_pos_deg"))
        {
            max_pos = ((Radians)Degrees(joints[i]["max_pos_deg"])).Value();
        } else {
            max_pos = joints[i]["max_pos"];
        }

        if (joints[i].hasMember("min_rpm"))
        {
            min_speed = ((Radians_Per_Sec)RPM(joints[i]["min_rpm"])).Value();
        } else {
            min_speed = joints[i]["min_speed"];
        }
        if (joints[i].hasMember("default_rpm"))
        {
            default_speed = ((Radians_Per_Sec)RPM(joints[i]["default_rpm"])).Value();
        } else {
            default_speed = joints[i]["default_speed"];
        }
        if (joints[i].hasMember("max_rpm"))
        {
            max_speed = ((Radians_Per_Sec)RPM(joints[i]["max_rpm"])).Value();
        } else {
            max_speed = joints[i]["max_speed"];
        }

        double waypoint_tolerance = joints[i]["waypoint_tolerance"];
        double goal_tolerance = joints[i]["goal_tolerance"];

        auto handle = hw_->getHandle(name);

        JointData joint = JointData(name, handle, min_pos, max_pos, min_speed, 
                    default_speed, max_speed, waypoint_tolerance, goal_tolerance);
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
        std::lock_guard<std::mutex> path_lock(joint_path_mutex_);

        if (!path_complete_)
        {
            // if current waypoint has been reached, load the next one
            if (waypoint_reached_)
            {
                waypoint_reached_ = false;
                // not the last waypoint in the current path
                if (idx_ < path_.points.size())
                {
                    ++idx_;
                    for (int i = 0; i < path_.joint_names.size(); ++i)
                    {
                        auto& joint = joint_map_.at(path_.joint_names[i].c_str());
                        // enable and set waypoint for this joint
                        joint.enabled = true;
                        joint.current_wp_end_pos = path_.points[idx_-1].positions[i];
                    }
                }
                else // last waypoint reached
                {
                    path_complete_ = true;
                }
            }

            // find joint with largest normalized position difference
            // also calculate waypoint completion ratio
            std::string max_distance_joint = "";
            double max_distance = -1.0;
            for (auto &joint_pair : joint_map_)
            {
                auto &joint = joint_pair.second;
                if (!joint.enabled) // joint not specified in current trajectory
                {
                    continue;
                }

                // store current joint position, for action feedback
                joint.current_pos = joint.handle.getPosition();

                // joint has reached waypoint
                if (fabs(joint.current_wp_end_pos - joint.current_pos) < joint.waypoint_tolerance)
                {
                    continue;
                }

                double distance = fabs(joint.current_wp_end_pos - joint.current_pos) 
                        / fabs(joint.max_pos - joint.min_pos); // normalize position difference
                if (distance > max_distance)
                {
                    max_distance = distance;
                    max_distance_joint = joint_pair.first;
                }
            }

            // all joint have reached current waypoint
            if (max_distance_joint.empty())
            {
                waypoint_reached_ = true;
                return;
            }

            // get joint which has the largest normalized position difference("max joint")
            JointData& max_joint = joint_map_.at(max_distance_joint);
            // get max scaled speed for the "max joint"               
            double max_speed = max_joint.max_speed;
            // use position difference and max_speed to get the time required to reach waypoint
            double wp_time = fabs(max_joint.current_wp_end_pos - max_joint.current_pos)/max_speed;

            // calculate speed for each joint using common wp_time
            // but individual postion differences
            for (auto& joint_pair : joint_map_)
            {
                auto& joint = joint_pair.second;
                if (!joint.enabled) // joint not specified in current trajectory
                {
                    // hold current position
                    joint.handle.setCommand(joint.handle.getPosition(), joint.default_speed);
                    continue;
                }

                if (joint_pair.first == max_distance_joint) // handle "max joint"
                {
                    joint.handle.setCommand(joint.current_wp_end_pos, max_speed);
                    continue;
                }

                double speed = fabs(joint.current_wp_end_pos-joint.current_pos) / wp_time;

                // limit speed to joint limits
                if (speed<joint.min_speed)
                {
                    speed = joint.min_speed;
                }
                else if (speed>joint.max_speed)
                {
                    speed = joint.max_speed;
                }

                joint.handle.setCommand(joint.current_wp_end_pos, speed);
                joint.last_speed = speed;
            }
        }
        else // path complete
        {
            int joints_not_reached_goal = 0; 

            // hold the goal position
            for (auto& joint_pair : joint_map_)
            {
                auto& joint = joint_pair.second;
                if (!joint.enabled) // joint not specified in current trajectory
                {
                    // hold current position
                    joint.handle.setCommand(joint.handle.getPosition(), joint.default_speed);
                    continue;
                }

                joint.current_pos = joint.handle.getPosition();
                if (fabs(joint.current_pos - joint.current_wp_end_pos) < joint.goal_tolerance)
                {
                    ++joints_not_reached_goal;
                }
                double speed = joint.default_speed;
                if (joint.last_speed > joint.min_speed)
                {
                    speed = joint.last_speed;
                }
                joint.handle.setCommand(joint.current_wp_end_pos, speed);
            }

            if (joints_not_reached_goal == 0)
            {
                goal_reached_ = true;
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
    if (isRunning())
    {
        return;
    }

     trajectory_msgs::JointTrajectory path; // empty path
    _lockAndReplacePath(path);

    has_goal = false;
}

void JointPathTrackingController::stopping(const ros::Time &time)
{
    std::lock_guard<std::mutex> preempt_lock(preempt_mutex_);
    preempted_ = true;

    if (this->isStopped())
    {
        return;
    }

    trajectory_msgs::JointTrajectory path; // empty path
    _lockAndReplacePath(path);

    preempted_ = false;
    has_goal = false;
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

void JointPathTrackingController::_lockAndReplacePath(const trajectory_msgs::JointTrajectory& path)
{
    std::lock_guard<std::mutex> path_lock(joint_path_mutex_);

    // reset all joint goals
    for (auto& joint_pair : joint_map_)
    {
        joint_pair.second.enabled = false;
        joint_pair.second.last_speed = -1.0;
    }

    idx_ = 0;
    path_ = path;
    waypoint_reached_ = true; // should be true at the start of each path
    path_complete_ = false;
    goal_reached_ = false;
}

void JointPathTrackingController::_executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
    // check if trajectory is valid
    if (goal->trajectory.joint_names.empty() // no joint names
        || goal->trajectory.points.empty() // no joint points 
        || goal->trajectory.points[0].positions.empty()) // positions vector is empty
    {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = result.INVALID_GOAL;
        as_.setAborted(result);
        return;
    }

    // check if all joints specified in the goal are present
    for (int i = 0; i < goal->trajectory.joint_names.size(); ++i)
    {
        if (joint_map_.find(goal->trajectory.joint_names[i]) == joint_map_.end())
        {
            control_msgs::FollowJointTrajectoryResult result;
            result.error_code = result.INVALID_JOINTS;
            as_.setAborted(result);
            return;
        }
    }

    // preempt and replace existing path
    {
        std::lock_guard<std::mutex> preempt_lock(preempt_mutex_);
        preempted_ = true;

        _lockAndReplacePath(goal->trajectory);

        preempted_ = false;
        has_goal = true;
    }

    ros::Rate rate(2);
    while (ros::ok() && !as_.isPreemptRequested())
    {
        if (goal_reached_)
        {
            control_msgs::FollowJointTrajectoryResult result;
            result.error_code = result.SUCCESSFUL;
            as_.setSucceeded(result);
            return;
        }
        else
        {
            control_msgs::FollowJointTrajectoryFeedback feedback;
            feedback.joint_names = path_.joint_names;

            size_t size = path_.joint_names.size();
            feedback.actual.positions.resize(size);
            feedback.desired.positions.resize(size);
            feedback.error.positions.resize(size);

            for(int i = 0; i<path_.joint_names.size(); ++i)
            {
                auto& joint = joint_map_.at(path_.joint_names[i]);

                feedback.actual.positions[i] = joint.current_pos;
                feedback.desired.positions[i] = joint.current_wp_end_pos;
                feedback.error.positions[i] = joint.current_wp_end_pos - joint.current_pos;
            }

            as_.publishFeedback(feedback);
        }

        rate.sleep();
    }
}

PLUGINLIB_EXPORT_CLASS(jpt_controller::JointPathTrackingController, controller_interface::ControllerBase)