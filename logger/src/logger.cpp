#include "logger.hpp"

void logger::sensorCallback(const ir_sensors::IRDists msg){
	
}

void logger::isTurningCallback(const std_msgs::Bool msg){
	ROS_INFO("GOT MESSAGE %d",msg.data);
}

void logger::encoderCallback(const ras_arduino_msgs::Encoders enc){
	
}

void logger::runNode(){
	ros::Rate loop_rate(10);	//10 Hz
	/*while (ros::ok())			//main loop of this code
	{
		geometry_msgs::Twist msg;	//for controlling the motor
		
		msg.linear.x = 0.0;
		msg.angular.y = 179;
		msg.angular.z = 0.0;
		
		pub_motor.publish(msg);		//pub to motor

		ros::spinOnce();
		loop_rate.sleep();
	}*/
	while(1){}
}

logger::logger(int argc, char *argv[]){
	ros::init(argc, argv, "logger");	//name of node
	ros::NodeHandle handle;					//the handle
	
	//pub_motor = handle.advertise<geometry_msgs::Twist>("/motor3/twist", 1000);
	sub_sensor = handle.subscribe("/ir_sensors/IRDists", 1000, &logger::sensorCallback, this);
	sub_isTurning = handle.subscribe("/motor3/is_turning", 1, &logger::isTurningCallback, this);
	sub_encoder = handle.subscribe("/arduino/encoders", 1, &logger::encoderCallback, this);
	
	runNode();
}

int main(int argc, char *argv[]) 
{
    logger logger(argc, argv);
}
