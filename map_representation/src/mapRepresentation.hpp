#include <ros/ros.h>
#include <rosutil/rosutil.hpp>
#include <utility>
#include <limits>
#include <mapping_msgs/LineVector.h>
#include <nav_msgs/OccupancyGrid.h>
#include <occupancy_grid_utils/coordinate_conversions.h>

struct MinMaxXY {
    MinMaxXY(float _minX, float _maxX, float _minY, float _maxY) : minX(_minX), maxX(_maxX), minY(_minY), maxY(_maxY){}
    float minX, maxX, minY, maxY;
};


class MapRepresentation {
public:
    MapRepresentation(int argc, char *argv[]);
private:
    ros::Subscriber line_sub;

    mapping_msgs::LineVector lines;
    nav_msgs::OccupancyGrid grid;
    
    void runNode();
    void lineCallback(const mapping_msgs::LineVector& msg);

    std::pair<float, float> findLineBoundSize(mapping_msgs::LineVector lines);
    MinMaxXY lineMinMax(mapping_msgs::Line l);
};

