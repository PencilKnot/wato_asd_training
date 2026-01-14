#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cmath>
#include <vector>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void setParameters(double distance_threshold, int map_width, int map_height, double map_resolution);
    const std::vector<int8_t>& getMap() const;
    int getWidth() const { return map_width; }
    int getHeight() const { return map_height; }
    double getResolution() const { return map_resolution; }
    double getOriginX() const { return origin_x; }
    double getOriginY() const { return origin_y; }
    void setLastPosition(double x, double y) { last_x = x; last_y = y; }
    void setYaw(double yaw_val) { yaw = yaw_val; }
    void setCostmap(const nav_msgs::msg::OccupancyGrid& costmap) { latest_costmap_ = costmap; }
    void setCostmapUpdated(bool val) { costmap_updated_ = val; }
    void setShouldUpdateMap(bool val) { should_update_map_ = val; }
    double getLastX() const { return last_x; }
    double getLastY() const { return last_y; }
    double getYaw() const { return yaw; }
    double getDistanceThreshold() const { return distance_threshold; }
    bool getCostmapUpdated() const { return costmap_updated_; }
    bool getShouldUpdateMap() const { return should_update_map_; }
    const nav_msgs::msg::OccupancyGrid& getLatestCostmap() const { return latest_costmap_; }
        void integrateCostmap(const nav_msgs::msg::OccupancyGrid& latest_costmap,
                          double last_x, double last_y, double yaw);

  private:
    rclcpp::Logger logger_;

    double distance_threshold;
    int map_width, map_height;
    double map_resolution;
    double origin_x, origin_y;
    std::vector<int8_t> global_map;
    
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    double last_x = 0.0, last_y = 0.0;
    double yaw = 0.0;
    bool costmap_updated_ = false, should_update_map_ = false;
};

}  

#endif  
