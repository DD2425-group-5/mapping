#include <ros/ros.h>
#include "rosutil/rosutil.hpp"
#include "hardware_msgs/Odometry.h"
#include "hardware_msgs/IRDists.h"
#include "controller_msgs/Turning.h"
#include "vision_msgs/object_found.h"

class Topological {
public:
    Topological(int argc, char *argv[]);
    void runNode();
private:
    hardware_msgs::Odometry latestOdom;
    hardware_msgs::IRDists latestIRDist;
    vision_msgs::object_found latestObject;

    ros::Subscriber sub_irdist;
    ros::Subscriber sub_odometry;
    ros::Subscriber sub_controlInfo;
    ros::Subscriber sub_objDetect;

    bool gotObject;
    bool turning;

    void odomCallback(const hardware_msgs::Odometry& msg);
    void irCallback(const hardware_msgs::IRDists& msg);
    void turnCallback(const controller_msgs::Turning& msg);
    void detectCallback(const vision_msgs::object_found& msg);
};
