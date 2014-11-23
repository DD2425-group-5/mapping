#include "mapRepresentation.hpp"

// Shorten the grid utils namespace
namespace ocutil = occupancy_grid_utils;

MapRepresentation::MapRepresentation(int argc, char *argv[]){
    ros::init(argc, argv, "map_representation");
    ros::NodeHandle handle;
    std::string lineTopic;
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/segment_stitching/published/stitched_line_topic",
                      lineTopic);
    
    ROSUtil::getParam(handle, "/map_representation/grid_resolution", grid.info.resolution);

    line_sub = handle.subscribe(lineTopic, 1, &MapRepresentation::lineCallback, this);
}

void MapRepresentation::runNode(){
    
}

void MapRepresentation::lineCallback(const mapping_msgs::LineVector& msg){
    lines = msg;
}

std::pair<float, float> MapRepresentation::findLineBoundSize(mapping_msgs::LineVector lineVec) {
    std::vector<mapping_msgs::Line> lines=lineVec.lines;

    float minX = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();
    for(size_t lineIterator = 0; lineIterator<lines.size(); lineIterator++){
        MinMaxXY minmax = lineMinMax(lines[lineIterator]);
        if (minmax.minX < minX){
            minX = minmax.minX;
        }
        if (minmax.maxX > maxX){
            maxX = minmax.maxX;
        }
        if (minmax.minY < minY){
            minY = minmax.minY;
        }
        if (minmax.maxY > maxY){
            maxY = minmax.maxY;
        }
    }

    float L1normX = maxX - minX;
    float L1normY = maxY - minY;

    return std::pair<float, float> (L1normX, L1normY);
}

MinMaxXY MapRepresentation::lineMinMax(mapping_msgs::Line l){
    float minX = l.start.x < l.end.x ? l.start.x : l.end.x;
    float maxX = l.start.x > l.end.x ? l.start.x : l.end.x;
    float minY = l.start.y < l.end.y ? l.start.y : l.end.y;
    float maxY = l.start.y > l.end.y ? l.start.y : l.end.y;

    return MinMaxXY(minX, maxX, minY, maxY);
}



void MapRepresentation::projectLineOntoGrid(mapping_msgs::Line lineToProject){
    ocutil::RayTraceIterRange iterators = occupancy_grid_utils::rayTrace( grid.info,
                                                                                    	lineToProject.start,
                                                                                     	lineToProject.end,
                                                                                        false,
                                                                                        false                );
                                       
    ocutil::RayTraceIterator iterator= iterators.first;
    ocutil::RayTraceIterator iteratorEnd= iterators.second;
    
    for(; !(iterator == iteratorEnd); iterator++ ){
        
        ocutil::index_t unfathomableIndex = occupancy_grid_utils::cellIndex	( grid.info,
                                                                              *iterator  );
        grid.data[unfathomableIndex] = 100;
    }
}


int main(int argc, char *argv[])
{
    MapRepresentation(argc, argv);
}
