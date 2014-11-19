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
    
}

void SegmentStitching::runNode(){
    
}

/**
 * Use RANSAC to extract the lines in each segment. Need to get line segments,
 * not just the line equations. Return two points representing each line?
 */
void SegmentStitching::extractLinesInSegment(mapping_msgs::MapSegment segment){
    
}

/**
 * Convert a segment point into a set of points corresponding to measurements of
 * walls in the maze. No need to take into account any rotation at this point -
 * assume that the robot is moving in a perfectly straight line. Assume (0,0) is
 * the robot location at the start of the segment, and then compute the x,y
 * coordinates of where the IR reading is measured according to the sensor
 * positions and distances received.
 */
void SegmentStitching::segmentPointToMeasurements(mapping_msgs::SegmentPoint pt){
    
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
