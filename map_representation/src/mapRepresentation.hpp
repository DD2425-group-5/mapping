#include <ros/ros.h>
#include <rosutil/rosutil.hpp>
#include <mapping_msgs/LineVector.h>

class MapRepresentation {
public:
    MapRepresentation(int argc, char *argv[]);
private:
    ros::Subscriber line_sub;
    
    void runNode();
    void lineCallback(const mapping_msgs::LineVector& msg);
};
