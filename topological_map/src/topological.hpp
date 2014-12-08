#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include "rosutil/rosutil.hpp"
#include "sysutil/sysutil.hpp"
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
    ~TopologicalMap();
private:
    // store the latest messages received
    hardware_msgs::Odometry latestOdom;
    hardware_msgs::IRDists latestIRDist;
    vision_msgs::object_found latestObject;

    ros::Subscriber sub_irdist;
    ros::Subscriber sub_odometry;
    ros::Subscriber sub_controlInfo;
    ros::Subscriber sub_objDetect;
    ros::Publisher pub_marker;
    ros::Publisher pub_map;
    
    // flags for indicating when to save a node
    bool gotObject;
    bool turning;
    // constructing map or just publishing it?
    bool construct;
    
    // path to the location where the bag file will be saved when the node is destroyed
    std::string bagDir;
    // topic to publish to in the rosbag
    std::string bagTopic;

    // keep track of the number of nodes created, use this to define the node
    // reference, which allows for a non-recursive message definition to be
    // used.
    int curNodeRef;
    
    mapping_msgs::NodeList nodes;
    // markerarray containing all the markers that have 
    visualization_msgs::MarkerArray currentMarkers;


    void odomCallback(const hardware_msgs::Odometry& msg);
    void irCallback(const hardware_msgs::IRDists& msg);
    void turnCallback(const controller_msgs::Turning& msg);
    void detectCallback(const vision_msgs::object_found& msg);

    void addNode(float x, float y, bool object, std::string label="");
    visualization_msgs::MarkerArray createMarkers();
    visualization_msgs::Marker createTextMarker(geometry_msgs::Point loc, std::string label);
    void saveMap();
    void runNode();
};
