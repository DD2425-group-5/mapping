#include <ros/ros.h>
#include <rosutil/rosutil.hpp>
#include <utility>
#include <cmath>
#include <limits>
#include <mapping_msgs/LineVector.h>
#include <mapping_msgs/SegmentLineVector.h>
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

    mapping_msgs::SegmentLineVector segLineVec;
    nav_msgs::OccupancyGrid grid;

    bool receivedLines;
    
    void runNode();
    void lineCallback(const mapping_msgs::SegmentLineVector& msg);

    void populateGrid();



    MinMaxXY findSegmentBounds(std::vector<mapping_msgs::LineVector> segmentLines);
    MinMaxXY findSegmentBounds(mapping_msgs::LineVector segmentLine);
    void setMinMaxFromStruct(MinMaxXY comp, float& minX, float& maxX, float& minY, float& maxY);
    void setCellsInBounds(geometry_msgs::Polygon bounds, signed char value);
    void setCellsOnLine(mapping_msgs::Line l, signed char value);
    geometry_msgs::Polygon boundsToPolygon(MinMaxXY bounds);
    MinMaxXY lineMinMax(mapping_msgs::Line l);
};

