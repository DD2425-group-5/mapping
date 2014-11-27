#include "segmentStorage.hpp"

SegmentStorage::SegmentStorage(int argc, char *argv[]){
    ros::init(argc, argv, "segment_storage");
    ros::NodeHandle handle;

    recordSegment = true;


    std::string odom_topic;
    ROSUtil::getParam(handle, "/topic_list/hardware_topics/odometry/published/odometry_topic",
                      odom_topic);
    sub_odometry = handle.subscribe(odom_topic, 1, &SegmentStorage::odomCallback, this);

    std::string ir_dist_topic;
    ROSUtil::getParam(handle, "/topic_list/hardware_topics/ir_sensors/published/ir_distance_topic",
                      ir_dist_topic);
    sub_irdist = handle.subscribe(ir_dist_topic, 1, &SegmentStorage::irCallback, this);

    
    std::string info_sub_topic;
    ROSUtil::getParam(handle, "/topic_list/controller_topics/wallfollower/published/turning_topic",
                      info_sub_topic);
    sub_controlInfo = handle.subscribe(info_sub_topic, 1000, &SegmentStorage::turnCallback, this);

    sub_objDetect = handle.subscribe("/vision/detection", 1000, &SegmentStorage::detectCallback, this);
    
    // The bag will use the segmentTopic string to define the topic which
    // segments are saved to
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/segment_storage/bag_segment_topic",
                      segmentTopic);

    // the rate is the rate at which messages are sent to the bag file
    ROSUtil::getParam(handle, "/segment_storage/rate", rate);

    std::string segment_save_dir;
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/segment_storage/segment_save_dir",
                      segment_save_dir);

    // Should we simulate output or use real values?
    ROSUtil::getParam(handle, "/segment_storage/simulated/simulate", simulate);

    if (simulate){
        ROSUtil::getParam(handle, "/segment_storage/simulated/s0mu", s0mu);
        ROSUtil::getParam(handle, "/segment_storage/simulated/s1mu", s1mu);
        ROSUtil::getParam(handle, "/segment_storage/simulated/s2mu", s2mu);
        ROSUtil::getParam(handle, "/segment_storage/simulated/s3mu", s3mu);
        ROSUtil::getParam(handle, "/segment_storage/simulated/distmu", distmu);

        ROSUtil::getParam(handle, "/segment_storage/simulated/s0std", s0std);
        ROSUtil::getParam(handle, "/segment_storage/simulated/s1std", s1std);
        ROSUtil::getParam(handle, "/segment_storage/simulated/s2std", s2std);
        ROSUtil::getParam(handle, "/segment_storage/simulated/s3std", s3std);
        ROSUtil::getParam(handle, "/segment_storage/simulated/diststd", diststd);
    }

    // Create the directory in which segments will be saved, if it does not exist
    if (!SysUtil::isDir(segment_save_dir)){
        SysUtil::makeDir(segment_save_dir);
    }

    std::string filePath = std::string(SysUtil::fullDirPath(segment_save_dir) +
                                       "segments_" +
                                       SysUtil::getDateTimeString() + ".bag");
    ROS_INFO("Segments will be saved to %s", filePath.c_str());
    // Create the rosbag in the segment directory
    segmentBag.open(filePath, rosbag::bagmode::Write);

    runNode();
}

SegmentStorage::~SegmentStorage(){
    // close the bag whent the destructor is called.
    endSegment(0);
    segmentBag.close();
}

void SegmentStorage::runNode(){
    ROS_INFO("Starting segment recording.");
    ros::Rate loopRate(rate);
    while (ros::ok()){
        ros::spinOnce();
        if (recordSegment){
            // The robot is moving in a straight line, so record points of
            // odometry and IR data
            if (simulate){
                ROS_INFO("---------- SIMULATED ----------");
                currentSegment.pointList.push_back(generateSimulatedPoint());
            } else {
                // first, add a point of the combined latest data received
                addPoint(latestOdom, latestIRDist, latestObject, gotObject);
                // then, clear all the data - in particular for latest object,
                // this might not be updated in the next loop, and we do not
                // want to keep it once it is added.
                hardware_msgs::Odometry clearOdom;
                hardware_msgs::IRDists clearIR;
                vision_master::object_found clearObject;
                latestOdom = clearOdom;
                latestIRDist = clearIR;
                latestObject = clearObject;
                gotObject = false;
            }
        }
        loopRate.sleep();
    }
}

void SegmentStorage::addPoint(hardware_msgs::Odometry odom,
                              hardware_msgs::IRDists ir,
                              vision_master::object_found obj,
                              bool gotObject){
    mapping_msgs::SegmentPoint p;
    p.distances = ir;
    p.odometry = odom;
    p.object = obj;
    p.gotObject = gotObject;
    if (p.gotObject){
        ROS_INFO("%%%%%%%%%%%%%%%%%%%% GOT OBJECT %%%%%%%%%%%%%%%%%%%%");
    }
    currentSegment.pointList.push_back(p);
}

mapping_msgs::SegmentPoint SegmentStorage::generateSimulatedPoint(){
    std::normal_distribution<double> s0(s0mu,s0std);
    std::normal_distribution<double> s1(s1mu,s1std);
    std::normal_distribution<double> s2(s2mu,s2std);
    std::normal_distribution<double> s3(s3mu,s3std);
    std::normal_distribution<double> dist(distmu, diststd);
    
    hardware_msgs::Odometry od;
    od.distanceTotal = simulatedDist;

    float moved = dist(generator);
    od.distanceFromLast = moved;
    simulatedDist += moved;
    
    hardware_msgs::IRDists ir;
    ir.s0 = s0(generator); // front left
    ir.s1 = s1(generator); // front right
    ir.s2 = s2(generator); // back left
    ir.s3 = s3(generator); // back rigth
    ir.s4 = 0.0; // cross left
    ir.s5 = 0.0; // cross right

    mapping_msgs::SegmentPoint sp;
    sp.distances = ir;
    sp.odometry = od;
    
    return sp;
}

void SegmentStorage::saveSegment(mapping_msgs::MapSegment seg){
    segmentBag.write(segmentTopic, ros::Time::now(), seg);
}

/**
 * Whenever the data from the IR sensors is received, we construct a
 * SegmentPoint and add it to the MapSegment.
 */
void SegmentStorage::irCallback(const hardware_msgs::IRDists::ConstPtr& msg){
    latestIRDist = *msg;
}

/**
 * Since the odometry rate is faster than that of the IR, we store the latest
 * message from the odometry. The value that is in this variable when the IR
 * callback happens is the odometry that is put into the segment.
 */
void SegmentStorage::odomCallback(const hardware_msgs::Odometry::ConstPtr& msg){
    // What is the best way to do this?
    latestOdom = *msg;
}

void SegmentStorage::detectCallback(const vision_master::object_found::ConstPtr& msg){
    ROS_INFO("******************** OBJECT RECEIVED ********************");
    latestObject = *msg;
    gotObject = true;
}

/**
 * Subscribe to turn information from the motor controller. recording only takes
 * place during straight line motions. Assume that true is sent when the turn
 * begins, and false when it ends, and when the robot is aligned to the wall.
 *
 * This callback controls when segments end and begin.
 */
void SegmentStorage::turnCallback(const controller_msgs::Turning& msg){
    if (msg.isTurning && recordSegment) { // starting turn - end current segment
        endSegment(msg.degrees);
    } else if (!msg.isTurning && !recordSegment){ // turn finished, start new segment
        ROS_INFO("Starting new segment");
        recordSegment = true;
        mapping_msgs::MapSegment ms;
        currentSegment = ms;
    }
}

void SegmentStorage::endSegment(float turnDegrees){
    ROS_INFO("Ending segment");
    recordSegment = false;
    if(turnDegrees == 90.0){
        currentSegment.turnDirection = currentSegment.LEFT_TURN;
    }
    else if(turnDegrees == -90.0){
        currentSegment.turnDirection = currentSegment.RIGHT_TURN;
    }
    else if(turnDegrees == 180.0){
        currentSegment.turnDirection = currentSegment.U_TURN;
    }
    segments.push_back(currentSegment);
    saveSegment(currentSegment);
    if (simulate){
        // reset the simulated distance on odometry for each segment to
        // mirror real behaviour
        simulatedDist = 0;
    }
}

int main(int argc, char *argv[]) {
    SegmentStorage s(argc, argv);
}
