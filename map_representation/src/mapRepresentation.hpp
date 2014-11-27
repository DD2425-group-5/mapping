#include <ros/ros.h>
#include <rosutil/rosutil.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <utility>
#include <iostream>
#include <cmath>
#include <limits>
#include <mapping_msgs/Line.h>
#include <mapping_msgs/LineVector.h>
#include <mapping_msgs/SegmentLineVector.h>
#include <mapping_msgs/Object.h>
#include <mapping_msgs/ObjectVector.h>
#include <mapping_msgs/SegmentObjectVector.h>
#include <mapping_msgs/StitchingResults.h>
#include "maputil/maputil.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Polygon.h>

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
    // subscriber for the lines and objects from segment stitching
    ros::Subscriber line_sub;
    ros::Subscriber object_sub;
    //tf::TransformBroadcaster br;
    ros::Publisher map_pub;

    // segment lines received from the segment stitching
    mapping_msgs::SegmentLineVector segmentLinesVector;
    mapping_msgs::SegmentObjectVector segmentObjectsVector;
    
    // map is stored in here
    nav_msgs::OccupancyGrid grid;

    // whether lines and objects have been received from segment stitching or not. Map is
    // not constructed until both of these are true.
    bool receivedLines;
    bool receivedObjects;
    
    void runNode();
    void lineCallback(const mapping_msgs::SegmentLineVector& msg);
    void objectCallback(const mapping_msgs::SegmentObjectVector& msg);
    void populateGrid();
    MinMaxXY findSegmentBounds(const std::vector<mapping_msgs::LineVector>& segmentLines);
    MinMaxXY findSegmentBounds(const mapping_msgs::LineVector& segmentLine);
    void setMinMaxFromStruct(const MinMaxXY& comp, float& minX, float& maxX, float& minY, float& maxY);
    void setCellsInBounds(const geometry_msgs::Polygon& bounds, signed char value);
    void setCellsOnLine(const mapping_msgs::Line& l, signed char value);
    geometry_msgs::Polygon boundsToPolygon(const MinMaxXY& bounds);
    MinMaxXY lineMinMax(const mapping_msgs::Line& l);
    void translateToOrigin(std::vector<mapping_msgs::LineVector>& segmentLines, const MinMaxXY& gridBounds);
    void alignToAxes(std::vector<mapping_msgs::LineVector>& segmentLines);
};

