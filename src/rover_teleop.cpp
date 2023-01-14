#include "rover_teleop.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "rover_teleop");
	ros::NodeHandle private_node("~");
	RoverTeleop RoverTeleop(private_node);
	ros::spin();
}