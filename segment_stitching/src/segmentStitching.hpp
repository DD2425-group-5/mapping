#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include "rosutil/rosutil.hpp"
#include "sysutil/sysutil.hpp"
#include "mapping_msgs/MapSegment.h"
#include "mapping_msgs/SegmentPoint.h"

class SegmentStitching {
public:
    SegmentStitching(int argc, char *argv[]);
private:
    rosbag::Bag segmentBag;
    std::vector<mapping_msgs::MapSegment> mapSegments;

    void runNode();
    void extractLinesInSegment(MapSegment segment);
    void segmentPointToMeasurements(mapping_msgs::SegmentPoint pt);
    void stitchSegmentLines();
};
