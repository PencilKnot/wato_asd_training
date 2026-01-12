#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void setParameters(double resolution, int width, int height, double inflationRadius);
    void initializeMap();
    void convertToGrid(double r, double a, int& x, int& y);
    void markObstacle(int x, int y);
    const std::vector<int8_t>& getMap() const;
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    double getResolution() const { return resolution; }
    int getOriginX() const { return originX; }
    int getOriginY() const { return originY; }

  private:
    rclcpp::Logger logger_;
    
    double resolution;
    int width, height;
    int originX, originY;
    double inflationRadius;
    std::vector<int8_t> map;
};
}  

#endif  