#include "mapRepresentation.hpp"
#include <nav_msgs/OccupancyGrid.h>

MapRepresentation::MapRepresentation(int argc, char *argv[]){
    ros::init(argc, argv, "map_representation");
    ros::NodeHandle handle;
    std::string lineTopic;
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/segment_stitching/published/stitched_line_topic",
                      lineTopic);
    line_sub = handle.subscribe(lineTopic, 1, &MapRepresentation::lineCallback, this);
}

void MapRepresentation::runNode(){
    
}

void MapRepresentation::lineCallback(const mapping_msgs::LineVector& msg){
    
}

int main(int argc, char *argv[])
{
    MapRepresentation(argc, argv);
}
