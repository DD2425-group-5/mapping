#include <ros/ros.h>
#include <rosutil/rosutil.hpp>
#include <utility>
#include <limits>
#include <mapping_msgs/LineVector.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Polygon.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/impl/ray_trace_iterator.h>
#include <occupancy_grid_utils/geometry.h>

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
    void projectLineOntoGrid(mapping_msgs::Line msg, signed char value);

    std::pair<float, float> findLineBoundSize(mapping_msgs::LineVector lines);

    void setCellsInBounds(geometry_msgs::Polygon bounds, signed char value);
    geometry_msgs::Polygon boundsFromLines(mapping_msgs::Line l1, mapping_msgs::Line l2);
    MinMaxXY lineMinMax(mapping_msgs::Line l);
};

