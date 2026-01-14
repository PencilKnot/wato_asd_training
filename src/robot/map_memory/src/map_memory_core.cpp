#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)  : logger_(logger) {}

void MapMemoryCore::setParameters(double distance_threshold, int map_width, int map_height, double map_resolution) {
    this->distance_threshold = distance_threshold;
    this->map_width = map_width;
    this->map_height = map_height;
    this->map_resolution = map_resolution;
    this->origin_x = (map_width / 2.0) * map_resolution;
    this->origin_y = (map_height / 2.0) * map_resolution;
    
    global_map.assign(map_width * map_height, 0);
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& latest_costmap,
                                      double last_x, double last_y, double yaw){
    int width = latest_costmap.info.width, height = latest_costmap.info.height;
    double local_resolution = latest_costmap.info.resolution;
    double local_origin_x = latest_costmap.info.origin.position.x, local_origin_y = latest_costmap.info.origin.position.y;

    for(int x = 0; x < width; x++){
      for(int y = 0; y < height; y++){
        int8_t cell = latest_costmap.data[y * width + x];

        // skip unknown
        if(cell < 0)
          continue;

        // x is corner of cell + 0.5 to get to center
        double local_x = local_origin_x + (x + 0.5) * local_resolution;
        double local_y = local_origin_y + (y + 0.5) * local_resolution;

        // transform to global map
        double transform_x = cos(yaw) * local_x - sin(yaw) * local_y + last_x;
        double transform_y = sin(yaw) * local_x + cos(yaw) * local_y + last_y;

        int global_x = std::round(transform_x / this->map_resolution + this->map_width / 2.0);
        int global_y = std::round(transform_y / this->map_resolution + this->map_height / 2.0);

        if(global_x < 0 || global_x >= this->map_width || global_y < 0 || global_y >= this->map_height)
          continue;
        
        int global_index = global_y * this->map_width + global_x;

        if(cell > global_map[global_index])
          global_map[global_index] = cell;
      }
    }
}

const std::vector<int8_t>& MapMemoryCore::getMap() const {
    return global_map;
}

} 
