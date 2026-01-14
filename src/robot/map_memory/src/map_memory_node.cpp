#include <chrono>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // parameters
  this->declare_parameter("distanceThreshold", 5.0);
  this->declare_parameter("map_width", 1000);
  this->declare_parameter("map_height", 1000);
  this->declare_parameter("map_resolution", 0.05);

  double distance_threshold = this->get_parameter("distanceThreshold").as_double();
  int map_width = this->get_parameter("map_width").as_int();
  int map_height = this->get_parameter("map_height").as_int();
  double map_resolution = this->get_parameter("map_resolution").as_double();

  map_memory_.setParameters(distance_threshold, map_width, map_height, map_resolution);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 1, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odometryCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::publishMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr grid){
  map_memory_.setCostmap(*grid);
  map_memory_.setCostmapUpdated(true);
}

void MapMemoryNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom){
  double x = odom->pose.pose.position.x;
  double y = odom->pose.pose.position.y;

  // get yaw
  auto q = odom->pose.pose.orientation;
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  map_memory_.setYaw(yaw);

  double distance = sqrt(pow(x - map_memory_.getLastX(), 2) + pow(y - map_memory_.getLastY(), 2));
  if(distance >= map_memory_.getDistanceThreshold()){
    map_memory_.setLastPosition(x, y);
    map_memory_.setShouldUpdateMap(true);
  }
}

void MapMemoryNode::publishMap(){
  if(map_memory_.getShouldUpdateMap() && map_memory_.getCostmapUpdated()){
    map_memory_.integrateCostmap(map_memory_.getLatestCostmap(), map_memory_.getLastX(), map_memory_.getLastY(), map_memory_.getYaw());
    
    nav_msgs::msg::OccupancyGrid globalMap;
    globalMap.header.frame_id = "sim_world";
    globalMap.header.stamp = this->now();
    globalMap.info.resolution = map_memory_.getResolution();
    globalMap.info.width = map_memory_.getWidth();
    globalMap.info.height = map_memory_.getHeight();
    globalMap.info.origin.position.x = -map_memory_.getOriginX();
    globalMap.info.origin.position.y = -map_memory_.getOriginY();
    globalMap.data = map_memory_.getMap();
    
    map_pub_->publish(globalMap);
    map_memory_.setShouldUpdateMap(false);
    map_memory_.setCostmapUpdated(false);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
