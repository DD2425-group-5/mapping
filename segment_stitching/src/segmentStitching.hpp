#include <ros/ros.h>
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <pcl/point_types.h>
#include "rosutil/rosutil.hpp"
#include "sysutil/sysutil.hpp"
#include "pclutil/pclutil.hpp"
#include "mapping_msgs/MapSegment.h"
#include "mapping_msgs/SegmentPoint.h"

class IRSensor {
public:
    IRSensor(std::string _name, float _x, float _y, float _z, float _rot) 
        : name(_name), x(_x), y(_y), z(_z), rotation(_rot){
        pclpt = pcl::PointXYZ(x, y ,z);
    }
    
    pcl::PointXYZ asPCLPoint(){
        return pclpt;
    }
    
    // should really be const, but can't get it to work properly
    std::string name;
    float x;
    float y;
    float z;
    float rotation;
private:
    IRSensor(){}
    friend std::ostream & operator<<(std::ostream &os, const IRSensor& p);
    pcl::PointXYZ pclpt;
};

std::ostream & operator<<(std::ostream &os, const IRSensor& s)
{
    char buffer[200];
    sprintf(buffer, "%s: (%f, %f, %f) rotated %f", s.name.c_str(), s.x, s.y, s.z, s.rotation);
    return os << buffer;
}

class SegmentStitching {
public:
    SegmentStitching(int argc, char *argv[]);
private:
    rosbag::Bag segmentBag;
    std::vector<mapping_msgs::MapSegment> mapSegments;
    std::vector<IRSensor> sensors;

    ros::Publisher segcloud_pub;

    void runNode();
    void extractLinesFromMeasurements(std::vector<pcl::PointXYZ> measurements);
    std::vector<pcl::PointXYZ> segmentPointToMeasurements(mapping_msgs::SegmentPoint pt);
    std::vector<pcl::PointXYZ> segmentToMeasurements(mapping_msgs::MapSegment segment);
    void stitchSegmentLines();
    void createOccupancyGrid();
    void populateSensorPositions(ros::NodeHandle handle);

};
