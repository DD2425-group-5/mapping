#include "topological.hpp"

int main(int argc, char *argv[])
{
    TopologicalMap tp(argc, argv);
}

TopologicalMap::TopologicalMap(int argc, char *argv[]) {
    ros::init(argc, argv, "topological");
    ros::NodeHandle handle;
    
    // Initialise all the subscribers
    std::string odom_sub_topic;
    ROSUtil::getParam(handle, "/topic_list/hardware_topics/odometry/published/odometry_topic",
                      odom_sub_topic);
    sub_odometry = handle.subscribe(odom_sub_topic, 1, &TopologicalMap::odomCallback, this);

    std::string ir_dist_sub_topic;
    ROSUtil::getParam(handle, "/topic_list/hardware_topics/ir_sensors/published/ir_distance_topic",
                      ir_dist_sub_topic);
    sub_irdist = handle.subscribe(ir_dist_sub_topic, 1, &TopologicalMap::irCallback, this);

    std::string info_sub_topic;
    ROSUtil::getParam(handle, "/topic_list/controller_topics/wallfollower/published/turning_topic",
                      info_sub_topic);
    sub_controlInfo = handle.subscribe(info_sub_topic, 1000, &TopologicalMap::turnCallback, this);

    sub_objDetect = handle.subscribe("/vision/detection", 1000, &TopologicalMap::detectCallback, this);

    // Initialise the publisher for markers and the map
    std::string marker_topic;
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/topological/published/marker_topic",
                      marker_topic);
    // latched publisher for markers
    pub_marker = handle.advertise<visualization_msgs::MarkerArray>(marker_topic, 1, true);

    std::string map_topic;
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/topological/published/map_topic",
                      map_topic);
    // latched publisher for the nodelist
    pub_map = handle.advertise<mapping_msgs::NodeList>(map_topic, 1, true);


    ROSUtil::getParam(handle, "/topological_map/merge_dist_threshold", MERGE_DIST_THRESHOLD);

    // The bag will use the bagtopic string to define the topic which
    // the map will be published to within the bag
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/topological/published/bag_topic", bagTopic);

    // if the bag variable is set, then we read from the bag and publish the map
    // contained, otherwise construct the map based on subscribed topics
    std::string bagFileName;
    ROSUtil::getParam(handle, "/topological/mapbag", bagFileName);



    // open the bag to allow intermediate maps to be saved the resulting bag
    // will be saved to this directory. We save bags even if a map has been
    // received, because we might add to the map.
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/topological/published/bag_dir", bagDir);

    std::string filePath = std::string(SysUtil::fullDirPath(bagDir) +
                                       "topmap_" +
                                       SysUtil::getDateTimeString() + ".bag");
    
    topBag.open(filePath, rosbag::bagmode::Write);

    if (bagFileName.compare("none") != 0){
        ROS_INFO("TopMap: Bag file received: %s  - extracting contained map", bagFileName.c_str());
        
        // open the bagfile received
        rosbag::Bag mapBag;
        mapBag.open(bagFileName, rosbag::bagmode::Read);
    
        // define the topics to read
        std::vector<std::string> topics;
        topics.push_back(bagTopic);
    
        // define a view onto the bag file - only interested in one topic
        rosbag::View view(mapBag, rosbag::TopicQuery(topics));

        // the most up to date map is the last one in the bag, so get the
        // iterator pointing to the last element.
        for (rosbag::View::iterator it = view.begin(); it != view.end(); it++){
            rosbag::MessageInstance mi = *it;
            nodes = *(mi.instantiate<mapping_msgs::NodeList>());
        }
        
        // need to start the references so that the references that were already
        // used in the generation of the map are not duplicated
        nextNodeRef = nodes.list.back().ref + 1;

        // publish the nodes and markers
        currentMarkers = createMarkers();
        pub_marker.publish(currentMarkers);
        pub_map.publish(nodes);
    } else {
        ROS_INFO("TopMap: No bagfile given - constructing map");
        construct = true;
    }

    traversedNodes.push_back(0); // the robot always starts at the first node added to the list

    // set bools to false
    gotObject = false;
    turning = false;

    runNode();
}

TopologicalMap::~TopologicalMap(){
    ROS_INFO("Topological map destructor");
    saveMap();
    topBag.close();
}

/**
 * Run the node, adding nodes to the topological map when messages are received
 */
void TopologicalMap::runNode(){
    // add a node at the start position
    mapping_msgs::Node start;
    start.x = 0.0f;
    start.y = 0.0f;
    start.label = "start";
    addNode(start);

    ros::Rate loopRate(10);
    bool added = false; // node added this loop?
    while (ros::ok()){
        // An object was detected
        if (gotObject){
            ROS_INFO("TopMap: Got object");
            addObject(latestOdom, latestObject);

            // reset the flag
            gotObject = false;
            added = true;
        }
        // robot is turning
        if (turning){
            ROS_INFO("TopMap: Got turn message");
            // create a node at the current odometry position
            mapping_msgs::Node odom;
            odom.x = latestOdom.totalX;
            odom.y = latestOdom.totalY;
            odom.object = false;

            addNode(odom);

            turning = false; // only create a single node when the message is received
            // update the markers being published
            added = true;
        }
        if (added){
            // update markers
            currentMarkers = createMarkers();
            // publish the updated markers and nodes
            pub_marker.publish(currentMarkers);
            pub_map.publish(nodes);
            saveMap();
            added = false;
            //ROS_INFO_STREAM("Current nodes: \n" << nodes);
        }
                    
        ros::spinOnce();
        loopRate.sleep();
    }

}

void TopologicalMap::saveMap(){
    topBag.write(bagTopic, ros::Time::now(), nodes);
}

void TopologicalMap::addObject(const hardware_msgs::Odometry& odom,
                               const vision_msgs::object_found& obj){

    // node at the odometry position when the object was detected
    mapping_msgs::Node odomPos;
    odomPos.x = odom.totalX;
    odomPos.y = odom.totalY;
    odomPos.object = false;

    // first, rotate the object around the origin. Since the coordinates are an
    // offset, we do not need to subtract anything from the point.
    std::pair<float, float> rotated = MathUtil::rotateAroundOrigin(obj.offset_x,
                                                                   obj.offset_y,
                                                                   odom.latestHeading);

    // object node at objectPos + odomPos, and set the node label to the name of
    // the object detected
    mapping_msgs::Node objPos;
    objPos.x = odom.totalX + rotated.first;
    objPos.y = odom.totalY + rotated.second;
    objPos.object = true;
    objPos.label = obj.id;

    addNode(odomPos);
    addNode(objPos);
}

/**
 * Add a node to the list of nodes in the topological map. If the previous node
 * is an object, then the node is connected to the previous non-object node.
 * Objects are connected in the same way, i.e. two objects detected at the same
 * position will be connected to that position. The function also checks in the
 * map for nodes which are in close proximity to this one, and if the distance
 * is below a certain threshold, the nodes are assumed to be the same.
 * Non-object nodes are not merged with object nodes, object nodes can be
 * merged.
 *
 * Returns true if the node was added, false if the node was merged
 */
bool TopologicalMap::addNode(mapping_msgs::Node& n){
    if (n.label.compare("start") == 0 && construct){ // hack to deal with the first node in a new map
        ROS_INFO("starting");
        n.ref = nextNodeRef++;
        nodes.list.push_back(n);
        return true;
    }

    // ROS_INFO("Last traversed node: %d", traversedNodes.back());

    // want to extract the node closest to the one that is being added
    int closestInd = 0;
    float minDist = nodeDistance(n, nodes.list[closestInd]);
    for (size_t i = 1; i < nodes.list.size(); i++) {
        mapping_msgs::Node other = nodes.list[i];
        // if the nodes are of different types, don't bother checking anything,
        // we can't merge them
        if (other.object != n.object){
            continue;
        }

        float dist = nodeDistance(n, other);
        if (dist < minDist){
            closestInd = i;
            minDist = dist;
        }
    }

    ROS_INFO("Distance to closest node(%d): %f, threshold: %f", nodes.list[closestInd].ref, minDist, MERGE_DIST_THRESHOLD);
    // if the minimum distance is below the threshold, then move the closest
    // node to the average of the new node and the old one. If the nodes are
    // objects, only merge if they have the same label.
    if (minDist < MERGE_DIST_THRESHOLD){
        mapping_msgs::Node& closest = nodes.list[closestInd];
        closest.x = (n.x + closest.x)/2;
        closest.y = (n.y + closest.y)/2;

        addLink(closest, nodes.list[traversedNodes.back()]);

        // merged the node instead of adding, so return false
        return false;
    }
    
    // if we end up here, the node is a new node, so set its reference and
    // increment the reference counter
    n.ref = nextNodeRef++;

    // link the node to the previous traversed node (will never link to objects)
    addLink(n, nodes.list[traversedNodes.back()]);

    // if the node added is not an object, then it is a turning node, or the
    // node added at the point where the robot was when the object was detected.
    // In this case, the reference of the new node should be added to the end of
    // the list of traversed nodes
    if (!n.object){
        traversedNodes.push_back(n.ref);
    }
    
    nodes.list.push_back(n);
    ROS_INFO_STREAM("TopMap: Added node " << n);
    std::cout << "Traversed nodes: ";
    for (size_t i = 0; i < traversedNodes.size(); i++) {
        std::cout << traversedNodes[i] << ", ";
    }
    std::cout << std::endl;

    return true;
}

/**
 * Add a link between two nodes, making sure that there are no duplicates. the
 * node lists are modified in place.
 */
void TopologicalMap::addLink(mapping_msgs::Node& n1, mapping_msgs::Node& n2){
    bool n2cont = false; // n2 contains n1 ref
    bool n1cont = false; // n1 contains n2 ref
    
    ROS_INFO("LINKING NODE %d TO NODE %d", n1.ref, n2.ref);
    
    // check the links of n2 for n1 ref
    for (size_t i = 0; i < n2.links.size(); i++) {
        if (n2.links[i] == n1.ref){
            n2cont = true;
            break;
        }
    }

    for (size_t i = 0; i < n1.links.size(); i++) {
        if (n1.links[i] == n2.ref){
            n1cont = true;
            break;
        }
    }

    // add reference to node 2 if node 1 list does not contain it
    if (!n1cont){
        n1.links.push_back(n2.ref);
    }

    // add reference to node 1 if node 2 list does not contain it
    if (!n2cont){
        n2.links.push_back(n1.ref);
    }
}

/**
 * Euclidean distance between two nodes.
 */
float TopologicalMap::nodeDistance(mapping_msgs::Node n1, mapping_msgs::Node n2){
    return sqrt(std::pow(n2.x - n1.x, 2) + std::pow(n2.y - n1.y, 2));
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

        //ROS_INFO("links in node %d (%s): %d", (int)i, dispText.c_str(), (int)curNode.links.size());
        
        geometry_msgs::Point linkedLoc;
        for (size_t i = 0; i < curNode.links.size(); i++) {
            //ROS_INFO_STREAM("" << curNode.links[i]);
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

visualization_msgs::Marker TopologicalMap::createTextMarker(const geometry_msgs::Point& loc, const std::string& label){
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
