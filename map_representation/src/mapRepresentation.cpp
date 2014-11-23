#include "mapRepresentation.hpp"


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

std::pair<float, float> findGridSize(mapping_msgs::LineVector lineVec) {
    std::vector<mapping_msgs::Line> lines=lineVec.lines;
    float minX = 0;
    float maxX = 0;
    float minY = 0;
    float maxY = 0;
    for(size_t lineIterator = 0; lineIterator<lines.size(); lineIterator++){
        //find min X
        if (lines[lineIterator].start.x < lines[lineIterator].end.x){
            //start has smaller X
            if (lines[lineIterator].start.x < minX){
                minX = lines[lineIterator].start.x;
            }
        }
        else{//end has smaller X
            if (lines[lineIterator].end.x < minX){
                minX = lines[lineIterator].end.x;
            }
        }
        
        //find min Y
        if (lines[lineIterator].start.y < lines[lineIterator].end.y){
            //start has smaller Y
            if (lines[lineIterator].start.y < minY){
                minY = lines[lineIterator].start.y;
            }
        }
        else{//end has smaller Y
            if (lines[lineIterator].end.y < minY){
                minY = lines[lineIterator].end.y;
            }
        }
        
        
         //find max X
        if (lines[lineIterator].start.x > lines[lineIterator].end.x){
            //start has bigger X
            if (lines[lineIterator].start.x > maxX){
                maxX = lines[lineIterator].start.x;
            }
        }
        else{//end has bigger X
            if (lines[lineIterator].end.x > maxX){
                maxX = lines[lineIterator].end.x;
            }
        }
        
        //find max Y
        if (lines[lineIterator].start.y > lines[lineIterator].end.y){
            //start has bigger Y
            if (lines[lineIterator].start.y > maxY){
                maxY = lines[lineIterator].start.y;
            }
        }
        else{//end has bigger Y
            if (lines[lineIterator].end.y > maxY){
                maxY = lines[lineIterator].end.y;
            }
        }
    }
    
    float L1normX = maxX - minX;
    float L1normY = maxY - minY;   
    return std::pair<float ,float> (L1normX, L1normY);
}


int main(int argc, char *argv[])
{
    MapRepresentation(argc, argv);
}
