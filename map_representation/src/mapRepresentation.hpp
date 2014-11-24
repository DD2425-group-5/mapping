#include <ros/ros.h>
#include <rosutil/rosutil.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <utility>
#include <iostream>
#include <cmath>
#include <limits>
#include <mapping_msgs/LineVector.h>
#include <mapping_msgs/SegmentLineVector.h>
#include "maputil/maputil.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Polygon.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/impl/ray_trace_iterator.h>
#include <occupancy_grid_utils/geometry.h>

struct MinMaxXY {
    MinMaxXY(float _minX, float _maxX, float _minY, float _maxY) : minX(_minX), maxX(_maxX), minY(_minY), maxY(_maxY){}
    float minX, maxX, minY, maxY;
    friend std::ostream& operator<<(std::ostream& os, const MinMaxXY& bound);
};

std::ostream& operator<<(std::ostream& os, const MinMaxXY& bound)
{
    os << "X bounds: [" << bound.minX << ", " << bound.maxX << "], ";
    os << "Y bounds: [" << bound.minY << ", " << bound.maxY << "], ";
        return os;
}


class MapRepresentation {
public:
    MapRepresentation(int argc, char *argv[]);
private:
    // subscriber for the lines created by the segment stitching
    ros::Subscriber line_sub;
    //tf::TransformBroadcaster br;
    ros::Publisher map_pub;

    // segment lines received from the segment stitching
    mapping_msgs::SegmentLineVector segLineVec;
    // map is stored in here
    nav_msgs::OccupancyGrid grid;

    // whether lines have been received from segment stitching or not. Map is
    // not constructed until this is true.
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
    void translateToOrigin(std::vector<mapping_msgs::LineVector>& segmentLines, MinMaxXY gridBounds);
};

