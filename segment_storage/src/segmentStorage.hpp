#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "rosutil/rosutil.hpp"
#include "hardware_msgs/IRDists.h"
#include "hardware_msgs/Odometry.h"
#include "mapping_msgs/MapSegment.h"
#include "mapping_msgs/SegmentPoint.h"

class SegmentStorage {
public:
    SegmentStorage(int argc, char *argv[]);
private:
    ros::Subscriber sub_irdist;
    ros::Subscriber sub_odometry;
    ros::Subscriber sub_controlInfo;

    bool recordSegment;
    mapping_msgs::MapSegment currentSegment;
    std::vector<mapping_msgs::MapSegment> segments; // not sure if needed - also storing rosbag
    
    hardware_msgs::Odometry latestOdom;
    hardware_msgs::IRDists latestIRDist;

    // callbacks
    void irCallback(const hardware_msgs::IRDists::ConstPtr& msg);
    void odomCallback(const hardware_msgs::Odometry::ConstPtr& msg);
    void turnCallback(const std_msgs::Bool msg);
    
    // main
    void runNode();

    // other functions
    void storePoint(hardware_msgs::Odometry odom, hardware_msgs::IRDists ir);
    void test();
};
