#include "topological.hpp"

int main(int argc, char *argv[])
{
    TopologicalMap tp(argc, argv);
}

TopologicalMap::TopologicalMap(int argc, char *argv[]) {
    ros::init(argc, argv, "topological");
    ros::NodeHandle handle;
    
    std::string odom_topic;
    ROSUtil::getParam(handle, "/topic_list/hardware_topics/odometry/published/odometry_topic",
                      odom_topic);
    sub_odometry = handle.subscribe(odom_topic, 1, &TopologicalMap::odomCallback, this);

    std::string ir_dist_topic;
    ROSUtil::getParam(handle, "/topic_list/hardware_topics/ir_sensors/published/ir_distance_topic",
                      ir_dist_topic);
    sub_irdist = handle.subscribe(ir_dist_topic, 1, &TopologicalMap::irCallback, this);

    
    std::string info_sub_topic;
    ROSUtil::getParam(handle, "/topic_list/controller_topics/wallfollower/published/turning_topic",
                      info_sub_topic);
    sub_controlInfo = handle.subscribe(info_sub_topic, 1000, &TopologicalMap::turnCallback, this);

    sub_objDetect = handle.subscribe("/vision/detection", 1000, &TopologicalMap::detectCallback, this);

    pub_marker = handle.advertise<visualization_msgs::MarkerArray>("/topological_map/markers", 1);
}

void TopologicalMap::runNode(){
    // add a node at the start position
    mapping_msgs::Node start;
    start.x = 0.0f;
    start.y = 0.0f;
    start.ref = curNodeRef++;
    start.label = "start";
    nodes.list.push_back(start);
    
    ros::Rate loopRate(10);
    while (ros::ok()){
        // An object was detected
        if (gotObject){
            // add object to map at objectPos + odomPos, and set the node label
            // to the name of the object detected
            addNode(latestOdom.totalX + latestObject.offset_x,
                    latestOdom.totalY + latestObject.offset_y, latestObject.id);
            
            // reset the flag
            gotObject = false;
            // update the markers being published
            currentMarkers = createMarkers();
        }
        // robot is turning
        if (turning){
            // create a node at the current odometry position
            addNode(latestOdom.totalX, latestOdom.totalY);
            turning = false; // only create a single node when the message is received
            // update the markers being published
            currentMarkers = createMarkers();
        }
        ros::spinOnce();
        pub_marker.publish(currentMarkers);
        loopRate.sleep();
    }
}

/**
 * Add a node to the list of nodes in the topological map. Assumes that each
 * node is connected to the one that came directly before it, and no other. If
 * the previous node is an object, then the node is connected to the previous
 * non-object node. Objects are connected in the same way.
 */
void TopologicalMap::addNode(float x, float y, std::string label){
    mapping_msgs::Node newNode;
    newNode.x = x;
    newNode.y = y;
    newNode.ref = curNodeRef++;
    if (nodes.list.back().object){
        // if the last node was an object, connect to the last non-object node
        // in the list.
        int nonObjectInd = nodes.list.size() - 1;
        while (nodes.list[nonObjectInd--].object);
        newNode.links.push_back(nodes.list[nonObjectInd].ref);
    } else {
        newNode.links.push_back(nodes.list.back().ref);
    }

    newNode.label = label;
    
    nodes.list.push_back(newNode);
}

visualization_msgs::MarkerArray TopologicalMap::createMarkers(){
    visualization_msgs::MarkerArray markers;

    // store node connections
    visualization_msgs::Marker lineList;
    lineList.header.frame_id = "map";
    lineList.ns = "lines";
    lineList.id = 0;
    lineList.type = visualization_msgs::Marker::LINE_LIST;
    lineList.action = visualization_msgs::Marker::ADD;
    lineList.scale.x = 0.1;
    lineList.color.a = 1.0;
    lineList.color.r = 0.0;
    lineList.color.g = 1.0;
    lineList.color.b = 0.0;

    // store markers for the positions of nodes
    visualization_msgs::Marker sphereList;
    sphereList.header.frame_id = "map";
    sphereList.ns = "nodes";
    sphereList.id = 1;
    sphereList.type = visualization_msgs::Marker::SPHERE_LIST;
    sphereList.action = visualization_msgs::Marker::ADD;
    sphereList.color.a = 1.0;
    sphereList.color.r = 0.0;
    sphereList.color.g = 1.0;
    sphereList.color.b = 0.0;

    bool prevObject; // is the previous node an object
    for (size_t i = 0; i < nodes.list.size(); i++) {
        mapping_msgs::Node curNode = nodes.list[i];
        // position of this node
        geometry_msgs::Point loc;
        loc.x = curNode.x;
        loc.y = curNode.y;

        // node positions are always the same
        sphereList.points.push_back(loc);

        // add a text marker for each location
        visualization_msgs::Marker text;
        text.header.frame_id = "map";
        text.ns = "labels";
        text.id = 3;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.points.push_back(loc);
        text.scale.z = 0.1;
        
        // populate the text string with either the object label or the node
        // reference
        std::string dispText;
        if (curNode.object){
            dispText = curNode.label;
        } else {
            char buf[10];
            sprintf(buf, "%d", curNode.ref);
            dispText = std::string(buf);
        }
        text.text = dispText;

        markers.markers.push_back(text);

        // don't need to add a line for the first point
        if (i == 0)
            continue;

        // location of previous point. Draw a line between that and current
        geometry_msgs::Point prevLoc;
        int prevInd = i - 1; // index of the previous object
        // handle case of previous node being an object - this node should
        // connect to the last non-object node
        if (prevObject){
            // decrement the non-object index until we find a node which is not
            // an object
            while(nodes.list[prevInd--].object);
        }
        
        // set the location of the previous node
        prevLoc.x = nodes.list[prevInd].x;
        prevLoc.y = nodes.list[prevInd].y;

        lineList.points.push_back(prevLoc);
        lineList.points.push_back(loc);

        // is the current node an object?
        prevObject = nodes.list[i].object ? true : false;
    }
    // populate the timestamp
    lineList.header.stamp = ros::Time();
    sphereList.header.stamp = ros::Time();
    
    markers.markers.push_back(lineList);
    markers.markers.push_back(sphereList);
    
    return markers;
}

void TopologicalMap::odomCallback(const hardware_msgs::Odometry& msg){
    latestOdom = msg;
}

void TopologicalMap::irCallback(const hardware_msgs::IRDists& msg){
    latestIRDist = msg;
}

void TopologicalMap::turnCallback(const controller_msgs::Turning& msg){
    if (msg.isTurning && !turning){
        turning = true; // set the flag so that we store a node
    }
}

void TopologicalMap::detectCallback(const vision_msgs::object_found& msg){
    latestObject = msg;
    gotObject = true;
}
