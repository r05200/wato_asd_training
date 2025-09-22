#include <algorithm>
#include <cmath>
#include <vector>
#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  {
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap",
        10,
        std::bind(&MapMemoryNode::costMapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered",
        10,
        std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MapMemoryNode::onTimer, this));
    width_ = 200;
    height_ = 200;
    resolution_ = 0.1;
    map_origin_x_ = -(width_ * resolution_) / 2.0;
    map_origin_y_ = -(height_ * resolution_) / 2.0;
    global_map_.resize(height_, std::vector<int8_t>(width_, 0));
    fuse_threshold_ = 1.5;
    last_fuse_x_ = 0.0;
    last_fuse_y_ = 0.0;
    robot_x_ = 0.0;
    robot_y_ = 0.0;
  }
}
void MapMemoryNode::costMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  double dx = robot_x_ - last_fuse_x_;
  double dy = robot_y_ - last_fuse_y_;
  double dist = std::sqrt(dx * dx + dy * dy);
  if (dist >= fuse_threshold_)
  {
    for (unsigned int y = 0; y < msg->info.height; ++y)
    {
      for (unsigned int x = 0; x < msg->info.width; ++x)
      {
        int8_t cost = msg->data[y * msg->info.width + x];
        if (cost > 0)
        {
          double wx = msg->info.origin.position.x + x * msg->info.resolution;
          double wy = msg->info.origin.position.y + y * msg->info.resolution;
          int gx = static_cast<int>((wx - map_origin_x_) / resolution_);
          int gy = static_cast<int>((wy - map_origin_y_) / resolution_);
          if (gx >= 0 && gx < width_ && gy >= 0 && gy < height_)
          {

            global_map_[gy][gx] = std::max(global_map_[gy][gx], cost);
          }
        }
      }
    }
    last_fuse_x_ = robot_x_;
    last_fuse_y_ = robot_y_;
  }
}
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
}
void MapMemoryNode::onTimer()
{
  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.header.stamp = this->now();
  map_msg.header.frame_id = "map";
  map_msg.info.resolution = resolution_;
  map_msg.info.width = width_;
  map_msg.info.height = height_;
  map_msg.info.origin.position.x = map_origin_x_;
  map_msg.info.origin.position.y = map_origin_y_;
  map_msg.info.origin.position.z = 0.0;
  map_msg.info.origin.orientation.w = 1.0;
  map_msg.data.resize(width_ * height_);
  for (int y = 0; y < height_; ++y)
  {
    for (int x = 0; x < width_; ++x)
    {
      map_msg.data[y * width_ + x] = global_map_[y][x];
    }
  }
  map_pub_->publish(map_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}

