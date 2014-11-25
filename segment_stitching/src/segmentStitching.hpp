#include <ros/ros.h>
#include <iostream>
#include <limits>
#include <math.h>
#include <utility>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/extract_indices.h>
#include "rosutil/rosutil.hpp"
#include "sysutil/sysutil.hpp"
#include "pclutil/pclutil.hpp"
#include "mathutil/mathutil.hpp"
#include "mapping_msgs/MapSegment.h"
#include "mapping_msgs/SegmentPoint.h"
#include "mapping_msgs/Line.h"
#include "mapping_msgs/LineVector.h"
#include "mapping_msgs/SegmentLineVector.h"
#include "mapping_msgs/Object.h"
#include "mapping_msgs/ObjectVector.h"
#include "mapping_msgs/SegmentObjectVector.h"
#include "mapping_msgs/StitchingResults.h"
#include "vision_master/object_found.h"


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
    friend std::ostream& operator<<(std::ostream &os, const IRSensor& p);
    pcl::PointXYZ pclpt;
};

std::ostream& operator<<(std::ostream &os, const IRSensor& s)
{
    char buffer[200];
    sprintf(buffer, "%s: (%f, %f, %f) rotated %f", s.name.c_str(), s.x, s.y, s.z, s.rotation);
    return os << buffer;
}

struct Line {
    Line(pcl::PointXYZ _start, pcl::PointXYZ _end) : start(_start), end(_end){}
    pcl::PointXYZ start;
    pcl::PointXYZ end;

    operator mapping_msgs::Line() const{
        mapping_msgs::Line l;
        l.start = PCLUtil::pclToGeomPoint(start);
        l.end = PCLUtil::pclToGeomPoint(end);
        return l;
    }

    friend std::ostream& operator<<(std::ostream &os, const Line& p);
};


std::ostream& operator<<(std::ostream &os, const Line& s)
{
    os << "Start: " << s.start;
    os << ", End: " << s.end;
    return os;
}

class SegmentStitching {
public:
    SegmentStitching(int argc, char *argv[]);
private:
    rosbag::Bag segmentBag;
    std::vector<mapping_msgs::MapSegment> mapSegments;
    std::vector<IRSensor> sensors;
    float ransacThreshold;
   
    pcl::PointXYZ rotatedStartPoint;
    pcl::PointXYZ rotatedEndPoint;
    ros::Publisher segcloud_pub;
    ros::Publisher linemarker_pub;
    ros::Publisher markerArray_pub;
    ros::Publisher stitchedResults_pub;

    void runNode();
    std::vector<Line> extractLinesFromMeasurements(pcl::PointCloud<pcl::PointXYZ>::Ptr measurements,
                                                   float ransacThreshold);
    Line extractLineFromMeasurements(pcl::PointCloud<pcl::PointXYZ>::Ptr measurements,
                                     float ransacThreshold,
                                     std::vector<int>* inliers);
    void segmentPointToMeasurements(const mapping_msgs::SegmentPoint& pt,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr measurements,
                                    mapping_msgs::ObjectVector& objects);
    
    void segmentToMeasurements(const mapping_msgs::MapSegment& segment,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr measurements,
                               mapping_msgs::ObjectVector& objects);
    
    Line rotateLine(const Line& lineToRotate, float angle);
    std::vector<std::vector<Line> > stitchSegmentLines(const std::vector<std::vector<Line> >& linesInSegments);
    void populateSensorPositions(ros::NodeHandle handle);
    void publishFinalMessages(const std::vector<std::vector<Line> >& lines,
                              const mapping_msgs::SegmentObjectVector& objects);
    
    void tmpPublish(pcl::PointCloud<pcl::PointXYZ>::Ptr cl, const std::vector<Line>& lines);
    visualization_msgs::Marker makeLineMarkers(const std::vector<Line>& lines, std::string markerRef);
    void publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cl);
    void publishSegmentLines(const std::vector<std::vector<Line> >& lines);
};
