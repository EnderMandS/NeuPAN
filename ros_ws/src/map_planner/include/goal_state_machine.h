#pragma once

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <map_planner/PlanPath.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <string>

class GoalStateMachine {
public:
  GoalStateMachine(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

private:
  void arriveCallback(const std_msgs::EmptyConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal);
  bool getRobotPose(geometry_msgs::PoseStamped& pose) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber arrive_sub_;
  ros::Subscriber goal_sub_;
  ros::ServiceClient plan_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::PoseStamped last_goal_;
  bool have_goal_{false};
  double goal_tolerance_{0.3};
  std::string plan_service_name_{"/map_planner/plan"};
};