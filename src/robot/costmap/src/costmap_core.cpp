#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::setParameters(double resolution, int width, int height, double inflationRadius) {
    this->resolution = resolution;
    this->width = width;
    this->height = height;
    this->originX = width/2;
    this->originY = height/2;
    this->inflationRadius = inflationRadius;
}

void CostmapCore::initializeMap(){
    map.assign(width*height, 0);
}

void CostmapCore::convertToGrid(double r, double a, int &x, int &y){
    x = ((r * cos(a)) / resolution) + originX;
    y = ((r * sin(a)) / resolution) + originY;
}

void CostmapCore::markObstacle(int x, int y){
    // make sure coords are within map
    if(x < 0 || x >= width || y < 0 || y >= height)
        return;

    map[y * width + x] = 100;

    // inflate obstacles according to radius
    int restoCell = inflationRadius / resolution;

    for(int i = -restoCell; i <= restoCell; i++){
        for(int j = -restoCell; j <= restoCell; j++){
            int inflatedX = x + i, inflatedY = y + j;
            int index = inflatedY * width + inflatedX;

            // do not assign cost to cell outside map
            if(inflatedX >= 0 && inflatedX < width && inflatedY >= 0 && inflatedY < height){
                double euclideanDist = sqrt(i * i + j * j) * resolution;
                
                // skip cells outside inflation radius
                if(euclideanDist > inflationRadius)
                    continue;

                double cost = 100 * (1-(euclideanDist/inflationRadius));

                // do not lower the cost in a cell
                if(cost > map[index])
                    map[index] = cost;
            }
        }
    }
}

const std::vector<int8_t>& CostmapCore::getMap() const {
    return map;
}
}