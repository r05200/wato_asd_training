#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node
{
public:
  CostmapNode();

  // Place callback function here
  void publishMessage();
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void convertToGrid(float range, float angle, int &x_grid, int &y_grid);
  void markObstacle(int x_grid, int y_grid);

private:
  robot::CostmapCore costmap_;
  // Place these constructs here
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::vector<std::vector<int>> costmap_grid_;
  float resolution_;
  int width_;
  int height_;
  float robot_y_;
  float robot_x_;
  double map_origin_x_;
  double map_origin_y_;
  double robot_yaw_;
};

#endif
