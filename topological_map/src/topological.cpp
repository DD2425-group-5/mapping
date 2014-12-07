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

    std::string marker_topic;
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/topological/published/marker_topic",
                      marker_topic);
    pub_marker = handle.advertise<visualization_msgs::MarkerArray>(marker_topic, 1);

    // The bag will use the bagtopic string to define the topic which
    // the map will be published to
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/topological/published/bag_topic", bagTopic);
    // the resulting bag will be saved to this directory
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/topological/published/bag_dir", bagDir);

    // set bools to false
    gotObject = false;
    turning = false;
    
    runNode();
}

TopologicalMap::~TopologicalMap(){
    ROS_INFO("Topological map destructor");
    saveMap();
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
            ROS_INFO("TopMap: Got object");
            // add object to map at objectPos + odomPos, and set the node label
            // to the name of the object detected
            addNode(latestOdom.totalX + latestObject.offset_x,
                    latestOdom.totalY + latestObject.offset_y, true, latestObject.id);
            
            // reset the flag
            gotObject = false;
            // update the markers being published
            currentMarkers = createMarkers();
        }
        // robot is turning
        if (turning){
            ROS_INFO("TopMap: Got turn message");
            // create a node at the current odometry position
            addNode(latestOdom.totalX, latestOdom.totalY, false);
            turning = false; // only create a single node when the message is received
            // update the markers being published
            currentMarkers = createMarkers();
        }
        ros::spinOnce();
        pub_marker.publish(currentMarkers);
        loopRate.sleep();
    }
}

void TopologicalMap::saveMap(){
    rosbag::Bag topBag;    
    std::string filePath = std::string(SysUtil::fullDirPath(bagDir) +
                                       "topmap_" +
                                       SysUtil::getDateTimeString() + ".bag");
    
    topBag.open(filePath, rosbag::bagmode::Write);
    topBag.write(bagTopic, ros::Time::now(), nodes);
    topBag.close();
}

/**
 * Add a node to the list of nodes in the topological map. Assumes that each
 * node is connected to the one that came directly before it, and no other. If
 * the previous node is an object, then the node is connected to the previous
 * non-object node. Objects are connected in the same way.
 */
void TopologicalMap::addNode(float x, float y, bool object, std::string label){
    mapping_msgs::Node newNode;
    newNode.x = x;
    newNode.y = y;
    newNode.ref = curNodeRef++;
    newNode.object = object;
    if (nodes.list.back().object){
        // if the last node was an object, connect to the last non-object node
        // in the list.
        int nonObjectInd = nodes.list.size() - 1;
        // go backwards through the list until the node is not an object
        for (; nodes.list[nonObjectInd].object; nonObjectInd--);
        
        // link the new node to the existing one
        newNode.links.push_back(nodes.list[nonObjectInd].ref);
        // link the last node to the new one
        nodes.list[nonObjectInd].links.push_back(newNode.ref);
    } else {
        // if the last node was not an object, link the node to the previous
        // node
        newNode.links.push_back(nodes.list.back().ref);
        // link the last node to the new one
        nodes.list.back().links.push_back(newNode.ref);
    }

    newNode.label = label;
    ROS_INFO_STREAM("TopMap: Added node " << newNode);

    nodes.list.push_back(newNode);
}

visualization_msgs::MarkerArray TopologicalMap::createMarkers(){
    visualization_msgs::MarkerArray markers;

    // store node connections
    visualization_msgs::Marker lineList;
    lineList.header.frame_id = "camera_link";
    lineList.ns = "lines";
    lineList.id = 0;
    lineList.type = visualization_msgs::Marker::LINE_LIST;
    lineList.action = visualization_msgs::Marker::ADD;
    lineList.scale.x = 0.01;
    lineList.color.a = 1.0;
    lineList.color.r = 0.3;
    lineList.color.g = 1.0;
    lineList.color.b = 0.3;

    // store markers for the positions of nodes
    visualization_msgs::Marker sphereList;
    sphereList.header.frame_id = "camera_link";
    sphereList.ns = "nodes";
    sphereList.id = 1;
    sphereList.type = visualization_msgs::Marker::SPHERE_LIST;
    sphereList.action = visualization_msgs::Marker::ADD;
    sphereList.scale.x = 0.1;
    sphereList.scale.y = 0.1;
    sphereList.scale.z = 0.1;
    sphereList.color.a = 1.0;
    sphereList.color.r = 1.0;
    sphereList.color.g = 1.0;
    sphereList.color.b = 1.0;

    for (size_t i = 0; i < nodes.list.size(); i++) {
        mapping_msgs::Node curNode = nodes.list[i];
        // position of this node
        geometry_msgs::Point loc;
        loc.x = curNode.x;
        loc.y = curNode.y;

        // node positions are always the same
        sphereList.points.push_back(loc);

        // add a text marker for each location
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
        visualization_msgs::Marker text = createTextMarker(loc, dispText);

        markers.markers.push_back(text);

        ROS_INFO("links in node %d (%s): %d", (int)i, dispText.c_str(), (int)curNode.links.size());
        
        geometry_msgs::Point linkedLoc;
        for (size_t i = 0; i < curNode.links.size(); i++) {
            ROS_INFO_STREAM("" << curNode.links[i]);
            // set the location of the previous node
            linkedLoc.x = nodes.list[curNode.links[i]].x;
            linkedLoc.y = nodes.list[curNode.links[i]].y;

            lineList.points.push_back(linkedLoc);
            lineList.points.push_back(loc);
        }
    }
    // populate the timestamp
    lineList.header.stamp = ros::Time();
    sphereList.header.stamp = ros::Time();
    
    markers.markers.push_back(lineList);
    markers.markers.push_back(sphereList);
    
    return markers;
}

visualization_msgs::Marker TopologicalMap::createTextMarker(geometry_msgs::Point loc, std::string label){
    visualization_msgs::Marker text;
    text.header.frame_id = "camera_link";
    text.ns = "label " + label;
    text.id = 3;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.position = loc;
    text.scale.z = 0.1;
    text.color.a = 1.0;
    text.text = label;

    return text;
}

void TopologicalMap::odomCallback(const hardware_msgs::Odometry& msg){
    latestOdom = msg;
}

void TopologicalMap::irCallback(const hardware_msgs::IRDists& msg){
    latestIRDist = msg;
}

void TopologicalMap::turnCallback(const controller_msgs::Turning& msg){
    ROS_INFO_STREAM("turn message is " << msg);
    if (msg.isTurning && !turning){
        turning = true; // set the flag so that we store a node
    }
}

void TopologicalMap::detectCallback(const vision_msgs::object_found& msg){
    latestObject = msg;
    gotObject = true;
}
