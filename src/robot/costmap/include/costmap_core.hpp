#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void setParameters(double resolution, int width, int height, double inflation_radius);
    void initializeMap();
    void convertToGrid(double r, double a, int& x, int& y);
    void markObstacle(int x, int y);
    const std::vector<int8_t>& getMap() const;
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    double getResolution() const { return resolution; }
    int getOriginX() const { return origin_x; }
    int getOriginY() const { return origin_y; }

  private:
    rclcpp::Logger logger_;
    
    double resolution;
    int width, height;
    int origin_x, origin_y;
    double inflation_radius;
    std::vector<int8_t> map;
};
}  

#endif  