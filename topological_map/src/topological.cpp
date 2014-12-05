#include "topological.hpp"

int main(int argc, char *argv[])
{
    Topological tp(argc, argv);
}

Topological::Topological(int argc, char *argv[]) {
    ros::init(argc, argv, "topological");
    ros::NodeHandle handle;
    
    std::string odom_topic;
    ROSUtil::getParam(handle, "/topic_list/hardware_topics/odometry/published/odometry_topic",
                      odom_topic);
    sub_odometry = handle.subscribe(odom_topic, 1, &Topological::odomCallback, this);

    std::string ir_dist_topic;
    ROSUtil::getParam(handle, "/topic_list/hardware_topics/ir_sensors/published/ir_distance_topic",
                      ir_dist_topic);
    sub_irdist = handle.subscribe(ir_dist_topic, 1, &Topological::irCallback, this);

    
    std::string info_sub_topic;
    ROSUtil::getParam(handle, "/topic_list/controller_topics/wallfollower/published/turning_topic",
                      info_sub_topic);
    sub_controlInfo = handle.subscribe(info_sub_topic, 1000, &Topological::turnCallback, this);

    sub_objDetect = handle.subscribe("/vision/detection", 1000, &Topological::detectCallback, this);
}

void Topological::runNode(){
    while (ros::ok()){
        if (gotObject){
            // add object to map at objectPos + odomPos
        }
        if (turning){
            // create a node at the current odometry position
            turning = false; // only create a single node when the message is received
        }
    }
}

void Topological::odomCallback(const hardware_msgs::Odometry& msg){
    latestOdom = *msg;
}

void Topological::irCallback(const hardware_msgs::IRDists& msg){
    
}

void Topological::turnCallback(const controller_msgs::Turning& msg){
    if (msg.isTurning && !turning){
        turning = true; // set the flag so that we store a node
    }
}

void Topological::detectCallback(const vision_msgs::object_found& msg){
    latestObject = *msg;
    gotObject = true;
}
