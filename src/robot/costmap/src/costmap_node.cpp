#include <chrono>
#include <memory>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("PSLidar"), costmap_(robot::CostmapCore(this->get_logger())), width_(100), height_(100), resolution_(0.1)
{
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar",
      10,
      std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered",
      10,
      std::bind(&CostmapNode::odomCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  resolution_ = 0.1;
  width_ = 100;
  height_ = 100;
  map_origin_x_ = -(width_ * resolution_) / 2.0;
  map_origin_y_ = -(height_ * resolution_) / 2.0;
  robot_x_ = 0.0;
  robot_y_ = 0.0;

  // Initialize the 2D array with zeros (free space)
  costmap_grid_.resize(height_, std::vector<int>(width_, 0));
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage()
{
  auto message = nav_msgs::msg::OccupancyGrid();
  message.header.stamp = this->now();
  message.header.frame_id = "map";
  message.info.resolution = resolution_;
  message.info.width = width_;
  message.info.height = height_;
  message.info.origin.position.x = map_origin_x_;
  message.info.origin.position.y = map_origin_y_;
  message.info.origin.position.z = 0.0;
  message.info.origin.orientation.w = 1.0;
  message.info.origin.orientation.x = 0.0;
  message.info.origin.orientation.y = 0.0;
  message.info.origin.orientation.z = 0.0;
  message.data.resize(width_ * height_);
  for (int y = 0; y < height_; ++y)
  {
    for (int x = 0; x < width_; ++x)
    {
      message.data[y * width_ + x] = costmap_grid_[y][x];
    }
  }

  costmap_pub_->publish(message);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{

  for (size_t i = 0; i < msg->ranges.size(); i++)
  {
    float range = msg->ranges[i];
    if (range < msg->range_max && range > msg->range_min)
    {
      int x_grid, y_grid;
      float angle = msg->angle_min + i * msg->angle_increment;

      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }
  publishMessage();
}
void CostmapNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  robot_yaw_ = yaw;
}

void CostmapNode::convertToGrid(float range, float angle, int &x_grid, int &y_grid)
{
  float world_angle = robot_yaw_ + angle;
  float x_map = robot_x_ + range * cos(world_angle);
  float y_map = robot_y_ + range * sin(world_angle);

  x_grid = static_cast<int>((x_map - map_origin_x_) / resolution_);
  y_grid = static_cast<int>((y_map - map_origin_y_) / resolution_);
}

void CostmapNode::markObstacle(int x_grid, int y_grid)
{
  double radius = 2;
  int int_radius = static_cast<int>(radius);
  if (x_grid < 0 || x_grid >= width_ || y_grid < 0 || y_grid >= height_)
  {
    return;
  }
  costmap_grid_[y_grid][x_grid] = 100;
  for (int y1 = y_grid - int_radius; y1 <= y_grid + int_radius; y1++)
  {

    for (int x1 = x_grid - int_radius; x1 <= x_grid + int_radius; x1++)
    {
      if (x1 < 0 || x1 >= width_ || y1 < 0 || y1 >= height_)
      {
        continue;
      }
      double dx = x1 - x_grid;
      double dy = y1 - y_grid;
      double distance = sqrt(dx * dx + dy * dy);
      if (distance <= radius)
      {
        double new_cost = 100 * (1 - distance / radius);
        int new_costint = static_cast<int>(new_cost);
        if (new_costint > costmap_grid_[y1][x1])
        {
          costmap_grid_[y1][x1] = new_costint;
        }
      }
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
