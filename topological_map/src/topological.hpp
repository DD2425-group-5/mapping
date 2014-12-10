#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <iostream>
#include "rosutil/rosutil.hpp"
#include "sysutil/sysutil.hpp"
#include "mathutil/mathutil.hpp"
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

    // below this threshold, nodes can be merged
    float MERGE_DIST_THRESHOLD;
    
    // flags for indicating when to save a node
    bool gotObject;
    bool turning;
    // constructing map or just publishing it?
    bool construct;
    
    rosbag::Bag topBag;
    // path to the location where the bag file will be saved when the node is destroyed
    std::string bagDir;
    // topic to publish to in the rosbag
    std::string bagTopic;

    // keep track of the number of nodes created, use this to define the node
    // reference, which allows for a non-recursive message definition to be
    // used.
    int nextNodeRef;

    // this should really be somewhere else - use this to keep track of the
    // previous nodes. Need to do this to ensure that nodes are added in the
    // correct place when we go over locations multiple times.
    std::vector<int> traversedNodes;
    
    mapping_msgs::NodeList nodes;

    // uses references to nodes so that they can be modified
    struct Edge {
        Edge(mapping_msgs::Node* _n1, mapping_msgs::Node* _n2){
            n1 = _n1;
            n2 = _n2;
            edge = MathUtil::Line(n1->x, n1->y, n2->x, n2->y);
        }
        mapping_msgs::Node* n1;
        mapping_msgs::Node* n2;
        MathUtil::Line edge;
    };

    // store the lines in the map as a vector of floats, where each four floats
    // define a line in the order x1,y1,x2,y2. Used to facilitate easier
    // and less expensive intersection computation.
    std::vector<Edge> lines;
    // markerarray containing all the markers that have 
    visualization_msgs::MarkerArray currentMarkers;


    void saveMap();
    void runNode();
    bool addNode(mapping_msgs::Node& n);
    void addObject(const hardware_msgs::Odometry& odom,
                   const vision_msgs::object_found& obj);
    float nodeDistance(mapping_msgs::Node n1, mapping_msgs::Node n2);
    void addLink(mapping_msgs::Node& n1, mapping_msgs::Node& n2);
    
    void odomCallback(const hardware_msgs::Odometry& msg);
    void irCallback(const hardware_msgs::IRDists& msg);
    void turnCallback(const controller_msgs::Turning& msg);
    void detectCallback(const vision_msgs::object_found& msg);
    bool checkLineIntersections(mapping_msgs::Node& addedNode, mapping_msgs::Node& previousNode);

    visualization_msgs::MarkerArray createMarkers();
    visualization_msgs::Marker createTextMarker(const geometry_msgs::Point& loc, const std::string& label);
};
