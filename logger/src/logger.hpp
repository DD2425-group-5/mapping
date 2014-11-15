#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "ras_arduino_msgs/Encoders.h"
#include "std_msgs/Bool.h"
#include "ir_sensors/IRDists.h"


class logger {
public:
	logger(int argc, char *argv[]);
	
private:
	ros::Subscriber sub_sensor;	//sub to get distance values
	ros::Publisher pub_motor;	//for the motor
	ros::Subscriber sub_encoder;// for encoder feedback
	ros::Subscriber sub_isTurning;// for encoder feedback
	
	void runNode();
	void sensorCallback(const ir_sensors::IRDists msg);
	void encoderCallback(const ras_arduino_msgs::Encoders feedback);
	void isTurningCallback(const std_msgs::Bool msg);
};
