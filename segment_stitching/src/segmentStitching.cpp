#include "segmentStitching.hpp"

SegmentStitching::SegmentStitching(int argc, char *argv[]) {
    ros::init(argc, argv, "segment_stitching");
    ros::NodeHandle handle;
    
    std::string segmentTopic;
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/segment_storage/bag_segment_topic",
                      segmentTopic);

    std::string segmentFile;
    ROSUtil::getParam(handle, "/segment_stitching/input_file",
                      segmentFile);

    segcloud_pub = handle.advertise<pcl::PointCloud<pcl::PointXYZ> >("/segment_stitching/segcloud", 1);

    // Initialise the locations of the IR sensors relative to the centre of the robot.
    populateSensorPositions(handle);

    // open the bag to read it
    segmentBag.open(segmentFile, rosbag::bagmode::Read);
    
    // define the topics to read
    std::vector<std::string> topics;
    topics.push_back(segmentTopic);
    
    // define a view onto the bag file - only interested in one topic
    rosbag::View view(segmentBag, rosbag::TopicQuery(topics));

    ROS_INFO("Reading segments from %s", segmentFile.c_str());
    // Iterate over messages in the bag
    for (rosbag::View::iterator it = view.begin(); it != view.end(); it++){
        // extract the messageInstance that the iterator points to
        rosbag::MessageInstance mi = *it;
        // Extract the segment from the bag
        mapping_msgs::MapSegment segment = *(mi.instantiate<mapping_msgs::MapSegment>());
        // Put it into the vector of segments
        mapSegments.push_back(segment);
    }

    ROS_INFO("Number of segments: %d", (int)mapSegments.size());

    runNode();
}

void SegmentStitching::runNode(){
    // Go through all the segments, extract measurements, and convert these to lines.
    for (size_t segment = 0; segment < mapSegments.size(); segment++) {
        ROS_INFO("Processing segment %d of %d", (int)(segment + 1), (int)(mapSegments.size()));
        // first, get the set of points which represent the measurements taken
        // in the segment
        std::vector<pcl::PointXYZ> measurements = segmentToMeasurements(mapSegments[segment]);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cl(new pcl::PointCloud<pcl::PointXYZ>);
        // insert the points into the cloud
        cl->insert(cl->begin(), measurements.begin(), measurements.end());
        // define the frame for the map
        // NEED TO PUBLISH THE MAP FRAME!
        cl->header.frame_id = "camera_link";
        
        ros::Rate loop(10);
        while(ros::ok()){
            segcloud_pub.publish(cl);
            ros::spinOnce();
            loop.sleep();
        }
        
        // Extract the lines from this segment using ransac
        extractLinesFromMeasurements(measurements);
    }

}

/**
 * Read data from the robot_info parameter and construct the offset coordinates
 * of the sensors
 */
void SegmentStitching::populateSensorPositions(ros::NodeHandle handle){
    int numSensors;
    ROSUtil::getParam(handle, "/robot_info/num_sensors", numSensors);
    std::string xSuffix;
    ROSUtil::getParam(handle, "/robot_info/sensor_x_suffix", xSuffix);
    std::string ySuffix;
    ROSUtil::getParam(handle, "/robot_info/sensor_y_suffix", ySuffix);
    std::string zSuffix;
    ROSUtil::getParam(handle, "/robot_info/sensor_z_suffix", zSuffix);
    std::vector<std::string> sensor_names;
    ROSUtil::getParam(handle, "/robot_info/sensor_names", sensor_names);
    
    for (size_t i = 0; i < sensor_names.size(); i++) {
        std::string xOffParam = std::string("/robot_info/" + sensor_names[i] + xSuffix);
        std::string yOffParam = std::string("/robot_info/" + sensor_names[i] + ySuffix);
        std::string zOffParam = std::string("/robot_info/" + sensor_names[i] + zSuffix);
        std::string rotParam = std::string("/robot_info/" + sensor_names[i] + "_rotation");

        float xoff;
        ROSUtil::getParam(handle, xOffParam, xoff);
        float yoff;
        ROSUtil::getParam(handle, yOffParam, yoff);
        float zoff;
        ROSUtil::getParam(handle, zOffParam, zoff);
        float rot;
        ROSUtil::getParam(handle, rotParam, rot);
        
        sensors.push_back(IRSensor(sensor_names[i], xoff, yoff, zoff, rot));
        ROS_INFO_STREAM(sensors[i]);
        std::cout << sensor_names[i] << " pcl point: " << sensors[i].asPCLPoint() << std::endl;
    }
}

/**
 * Convert a whole segment into a set of points corresponding to measurements
 * taken by IR sensors during the motion in the segment.
 */
std::vector<pcl::PointXYZ> SegmentStitching::segmentToMeasurements(mapping_msgs::MapSegment segment){
    // iterate over each point measurement in the segment
    std::vector<pcl::PointXYZ> measurements;
    ROS_INFO("Points in segment: %d", (int)segment.pointList.size());

    for (size_t point = 0; point < segment.pointList.size(); point++){

        // convert the IR distances at the point to a useful coordinate space
        std::vector<pcl::PointXYZ> pointMeasurements = segmentPointToMeasurements(segment.pointList[point]);
        // add the measurements to the vector that is being accumulated
        measurements.insert(measurements.end(), pointMeasurements.begin(),
                            pointMeasurements.end());
    }
    ROS_INFO("Measurements in segment: %d", (int)measurements.size());
    return measurements;
}

/**
 * Convert a segment point into a set of points corresponding to measurements of
 * walls in the maze. No need to take into account any rotation at this point -
 * assume that the robot is moving in a perfectly straight line. Assume (0,0) is
 * the robot location at the start of the segment, and then compute the x,y
 * coordinates of where the IR reading is measured according to the sensor
 * positions and distances received. Only consider the side facing sensors.
 */
std::vector<pcl::PointXYZ> SegmentStitching::segmentPointToMeasurements(mapping_msgs::SegmentPoint pt){
    std::vector<pcl::PointXYZ> measurements;
    
    hardware_msgs::IRDists dists = pt.distances;
    hardware_msgs::Odometry odom = pt.odometry;
    
    float distances[6] = {dists.s0, dists.s1, dists.s2, dists.s3, dists.s4, dists.s5};
    // The robot base has moved relative to the start of the segment
    pcl::PointXYZ odompt(0, odom.distanceTotal, 0);
    // only look at first 4 sensors, last 2 are front facing, assume either -90 or 90 rotation
    for (size_t i = 0; i < sensors.size() - 2; i++){
        IRSensor s = sensors[i];
        // naive way of getting point measurement - if sensor rotated -90,
        // subtract from x, otherwise add. The generated points will be rotated
        // later to match segment rotation if necessary
        if (s.rotation == -90) { // might be dangerous, rotation is a float
            distances[i] = -distances[i];
        }
        pcl::PointXYZ p = s.asPCLPoint() + odompt + pcl::PointXYZ(distances[i], 0, 0);
        // std::cout << "adding " << s.asPCLPoint() << ", " << pcl::PointXYZ(distances[i], 0, 0) << std::endl;
        // std::cout << "result: " << p << std::endl;
        measurements.push_back(p);
    }
    return measurements;
}

/**
 * Extract lines from a set of points using ransac
 */
void SegmentStitching::extractLinesFromMeasurements(std::vector<pcl::PointXYZ> measurements){
    
}

/**
 * Combine the lines extracted from segments into a single set of lines which
 * hopefully represent the walls in the maze. The first segment starts at 0,0
 * and other segments need to be rotated to correspond to the relative rotation
 * of the segment that precedes them, based on the turn made by the robot. Just
 * rotating the points that correspond to the line start and end and then
 * rotating around the point that ends the previous segment should work?
 * Transform the point we are rotating around to the origin, rotate the points,
 * then translate back.
 */
void SegmentStitching::stitchSegmentLines(){
    
}

/**
 * Create an occupancy grid from the stitched lines. Once these are extracted it
 * should be possible to simply travel along the line with a small step, and
 * populate all the points in the occupancy grid that the line passes through.
 */
void SegmentStitching::createOccupancyGrid(){
}


int main(int argc, char *argv[])
{
    SegmentStitching stitch(argc, argv);
}
