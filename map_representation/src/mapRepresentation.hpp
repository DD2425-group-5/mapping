#include <ros/ros.h>
#include <rosutil/rosutil.hpp>
#include <utility>
#include <mapping_msgs/LineVector.h>
#include <nav_msgs/OccupancyGrid.h>

class MapRepresentation {
public:
    MapRepresentation(int argc, char *argv[]);
private:
    ros::Subscriber line_sub;

    mapping_msgs::LineVector lines;
    nav_msgs::OccupancyGrid grid;
    
    void runNode();
    void lineCallback(const mapping_msgs::LineVector& msg);

    std::pair<float, float> findLineBoundSize(mapping_msgs::LineVector lines);
    
    
};
