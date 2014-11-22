#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <random>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "rosutil/rosutil.hpp"
#include "sysutil/sysutil.hpp"
#include "hardware_msgs/IRDists.h"
#include "hardware_msgs/Odometry.h"
#include "mapping_msgs/MapSegment.h"
#include "mapping_msgs/SegmentPoint.h"
#include "controller_msgs/Turning.h"

class SegmentStorage {
public:
    SegmentStorage(int argc, char *argv[]);
    ~SegmentStorage();
private:
    ros::Subscriber sub_irdist;
    ros::Subscriber sub_odometry;
    ros::Subscriber sub_controlInfo;

    // simulation
    std::default_random_engine generator;
    bool simulate;
    float simulatedDist;
    float s0mu, s0std;
    float s1mu, s1std;
    float s2mu, s2std;
    float s3mu, s3std;
    float distmu, diststd;

    bool recordSegment;
    mapping_msgs::MapSegment currentSegment;
    std::vector<mapping_msgs::MapSegment> segments; // not sure if needed - also storing rosbag
    
    hardware_msgs::Odometry latestOdom;
    hardware_msgs::IRDists latestIRDist;
    rosbag::Bag segmentBag;
    std::string segmentTopic;
    
    // callbacks
    void irCallback(const hardware_msgs::IRDists::ConstPtr& msg);
    void odomCallback(const hardware_msgs::Odometry::ConstPtr& msg);
    void turnCallback(const controller_msgs::Turning msg);
    
    // main
    void runNode();
    
    // other functions
    void addPoint(hardware_msgs::Odometry odom, hardware_msgs::IRDists ir);
    void saveSegment(mapping_msgs::MapSegment seg);
    void endSegment(float turnDegrees);
    mapping_msgs::SegmentPoint generateSimulatedPoint();
};
