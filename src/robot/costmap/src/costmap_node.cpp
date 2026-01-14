#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // parameters
  this->declare_parameter("resolution", 0.05);
  this->declare_parameter("width", 250);
  this->declare_parameter("height", 250);
  this->declare_parameter("inflationRadius", 1.0);

  double resolution = this->get_parameter("resolution").as_double();
  int width = this->get_parameter("width").as_int();
  int height = this->get_parameter("height").as_int();
  double inflation_radius = this->get_parameter("inflationRadius").as_double();

  // set up costmap core
  costmap_.setParameters(resolution, width, height, inflation_radius);

  laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserscanCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 1);

}

void CostmapNode::laserscanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){ 
  costmap_.initializeMap();

  // raycasting ig
  for(size_t i = 0; i < scan->ranges.size(); i++){
    // convert index of laser scan into physical angle
    double angle = scan->angle_min + i * scan->angle_increment;
    // get the distance at that angle
    double range = scan->ranges[i];
    if(range < scan->range_max && range > scan->range_min){
      int x_grid, y_grid;
      costmap_.convertToGrid(range, angle, x_grid, y_grid);
      costmap_.markObstacle(x_grid, y_grid);
    }
  }

  publishMap();
}

void CostmapNode::publishMap(){
  nav_msgs::msg::OccupancyGrid costMap;
  costMap.header.frame_id = "robot/chassis/lidar";
  costMap.header.stamp = this->now();
  costMap.info.resolution = costmap_.getResolution();
  costMap.info.width = costmap_.getWidth();
  costMap.info.height = costmap_.getHeight();
  costMap.info.origin.position.x = -costmap_.getOriginX() * costmap_.getResolution();
  costMap.info.origin.position.y = -costmap_.getOriginY() * costmap_.getResolution();
  costMap.data = costmap_.getMap();

  costmap_pub_->publish(costMap);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}