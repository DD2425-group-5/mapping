#include <ros/ros.h>
#include "rosutil/rosutil.hpp"
#include "hardware_msgs/Odometry.h"
#include "hardware_msgs/IRDists.h"
#include "controller_msgs/Turning.h"
#include "vision_msgs/object_found.h"
#include "mapping_msgs/Node.h"
#include "mapping_msgs/NodeList.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"

class TopologicalMap {
public:
    TopologicalMap(int argc, char *argv[]);
    void runNode();
private:
    hardware_msgs::Odometry latestOdom;
    hardware_msgs::IRDists latestIRDist;
    vision_msgs::object_found latestObject;

    ros::Subscriber sub_irdist;
    ros::Subscriber sub_odometry;
    ros::Subscriber sub_controlInfo;
    ros::Subscriber sub_objDetect;
    ros::Publisher pub_marker;
    
    bool gotObject;
    bool turning;

    // keep track of the number of nodes created, use this to define the node
    // reference, which allows for a non-recursive message definition to be
    // used.
    int curNodeRef;

    mapping_msgs::NodeList nodes;
    visualization_msgs::MarkerArray currentMarkers;


    void odomCallback(const hardware_msgs::Odometry& msg);
    void irCallback(const hardware_msgs::IRDists& msg);
    void turnCallback(const controller_msgs::Turning& msg);
    void detectCallback(const vision_msgs::object_found& msg);

    void addNode(float x, float y, std::string label="");
    visualization_msgs::MarkerArray createMarkers();
    
};
