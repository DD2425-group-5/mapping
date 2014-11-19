#include "segmentStorage.hpp"

SegmentStorage::SegmentStorage(int argc, char *argv[]){
    ros::init(argc, argv, "segment_storage");
    ros::NodeHandle handle;

    recordSegment = true;

    sub_irdist = handle.subscribe("/ir_sensors/dists", 1, &SegmentStorage::irCallback, this);
    sub_odometry = handle.subscribe("/odometry/odometry", 1, &SegmentStorage::odomCallback, this);
    
    std::string info_sub_topic;
    ROSUtil::getParam(handle, "/topic_list/controller_topics/motor3/published/bool_topic",
                      info_sub_topic);
    sub_controlInfo = handle.subscribe(info_sub_topic, 1000, &SegmentStorage::turnCallback, this);
    
    // The bag will use the segmentTopic string to define the topic which
    // segments are saved to
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/segment_storage/bag_segment_topic",
                      segmentTopic);

    std::string segment_save_dir;
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/segment_storage/segment_save_dir",
                      segment_save_dir);

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
    segmentBag.close();
}

void SegmentStorage::runNode(){
    ros::Rate loopRate(10);
    while (ros::ok()){
        ros::spinOnce();
        if (recordSegment){
            // The robot is moving in a straight line, so record points of
            // odometry and IR data
            addPoint(latestOdom, latestIRDist);
            //            currentSegment.pointList.push_back(mapping_msgs::SegmentPoint());
        }
        loopRate.sleep();
    }
}

void SegmentStorage::addPoint(hardware_msgs::Odometry odom, hardware_msgs::IRDists ir){
    mapping_msgs::SegmentPoint p;
    p.distances = ir;
    p.odometry = odom;
    
    currentSegment.pointList.push_back(p);
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

/**
 * Subscribe to turn information from the motor controller. recording only takes
 * place during straight line motions. Assume that true is sent when the turn
 * begins, and false when it ends, and when the robot is aligned to the wall.
 *
 * This callback controls when segments end and begin.
 */
void SegmentStorage::turnCallback(const std_msgs::Bool msg){
    if (msg.data && recordSegment) { // starting turn - end current segment
        ROS_INFO("Ending segment");
        recordSegment = false;
        segments.push_back(currentSegment);
        saveSegment(currentSegment);
    } else if (!msg.data && !recordSegment){ // turn finished, start new segment
        ROS_INFO("Starting new segment");
        recordSegment = true;
        mapping_msgs::MapSegment ms;
        currentSegment = ms;
    }
}

int main(int argc, char *argv[]) {
    SegmentStorage s(argc, argv);
}
