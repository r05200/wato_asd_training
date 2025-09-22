#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node
{
public:
  MapMemoryNode();

private:
  robot::MapMemoryCore map_memory_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void costMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onTimer();
  double robot_x_;
  double robot_y_;
  int width_;
  int height_;
  double resolution_;
  double map_origin_x_;
  double map_origin_y_;
  double last_fuse_x_;
  double last_fuse_y_;
  double fuse_threshold_;
  std::vector<std::vector<int8_t>> global_map_;
};

#endif
