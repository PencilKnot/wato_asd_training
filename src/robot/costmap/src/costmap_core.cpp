#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::setParameters(double resolution, int width, int height, double inflation_radius) {
    this->resolution = resolution;
    this->width = width;
    this->height = height;
    this->origin_x = width/2;
    this->origin_y = height/2;
    this->inflation_radius = inflation_radius;
}

void CostmapCore::initializeMap(){
    map.assign(width*height, 0);
}

void CostmapCore::convertToGrid(double r, double a, int &x, int &y){
    x = ((r * cos(a)) / resolution) + origin_x;
    y = ((r * sin(a)) / resolution) + origin_y;
}

void CostmapCore::markObstacle(int x, int y){
    // make sure coords are within map
    if(x < 0 || x >= width || y < 0 || y >= height)
        return;

    map[y * width + x] = 100;

    // inflate obstacles according to radius
    int res_to_cell = inflation_radius / resolution;

    for(int i = -res_to_cell; i <= res_to_cell; i++){
        for(int j = -res_to_cell; j <= res_to_cell; j++){
            int inflated_x = x + i, inflated_y = y + j;
            int index = inflated_y * width + inflated_x;

            // do not assign cost to cell outside map
            if(inflated_x >= 0 && inflated_x < width && inflated_y >= 0 && inflated_y < height){
                double euclidean_dist = sqrt(i * i + j * j) * resolution;
                
                // skip cells outside inflation radius
                if(euclidean_dist > inflation_radius)
                    continue;

                double cost = 100 * (1-(euclidean_dist/inflation_radius));

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