#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <map_planner/PlanPath.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <vector>

class MapPlanner {
public:
  MapPlanner(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal);
  bool getRobotPose(geometry_msgs::PoseStamped& pose) const;
  bool plan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path);
  bool planService(map_planner::PlanPath::Request& req, map_planner::PlanPath::Response& res);
  void inflateMap();
  void publishInflatedMap();
  bool worldToMap(const geometry_msgs::Point& point, int& mx, int& my) const;
  geometry_msgs::Point mapToWorld(int mx, int my) const;
  inline int toIndex(int mx, int my) const { return my * static_cast<int>(map_.info.width) + mx; }
  bool isFree(int index) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;
  ros::Publisher inflated_map_pub_;
  ros::ServiceServer plan_service_;
  bool publish_path_{true};
  std::string plan_service_name_{"plan"};

  nav_msgs::OccupancyGrid map_;
  std::vector<int8_t> inflated_data_;
  bool map_ready_{false};
  double inflation_radius_{0.25};
  int obstacle_threshold_{50};
  int inflation_cells_{1};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
