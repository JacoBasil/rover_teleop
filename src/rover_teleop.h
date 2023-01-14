#ifndef ROVER_TELEOP_H_
#define ROVER_TELEOP_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

constexpr uint8_t PS4_AXIS_STICK_LEFT_LEFTWARDS = 0;
constexpr uint8_t PS4_AXIS_STICK_LEFT_UPWARDS = 1;
constexpr uint8_t PS4_AXIS_STICK_RIGHT_LEFTWARDS = 2;
constexpr uint8_t PS4_AXIS_STICK_RIGHT_UPWARDS = 5;

class RoverTeleop {
public:
		RoverTeleop(ros::NodeHandle private_node);
private:
		ros::NodeHandle _node;
		ros::NodeHandle _private_node;
		ros::Subscriber _joy_sub;
		ros::Publisher _cmd_vel_pub;

		bool _last_zero_twist = true; 
		double _linear_speed_scale_x;
		double _linear_speed_scale_y;
		double _angular_speed_scale;
		
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
};

RoverTeleop::RoverTeleop(ros::NodeHandle private_node) :
		_private_node(private_node) {
		_private_node.param<double>("linear_speed_scale_x", _linear_speed_scale_x, 0.0);
		_private_node.param<double>("linear_speed_scale_y", _linear_speed_scale_y, 0.0);
		_private_node.param<double>("angular_speed_scale", _angular_speed_scale, 0.0);
		_cmd_vel_pub = _node.advertise<geometry_msgs::Twist>("/rover_mecanum_controller/cmd_vel", 1);
		_joy_sub = _node.subscribe<sensor_msgs::Joy>("joy", 10, &RoverTeleop::joyCallback, this);
		ROS_INFO("Rover teleop node: Start");
}

void RoverTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
		geometry_msgs::Twist twist;

		double twist_linear_x_vel =  _linear_speed_scale_x * joy->axes[PS4_AXIS_STICK_LEFT_UPWARDS];
        double twist_linear_y_vel =  _linear_speed_scale_y * joy->axes[PS4_AXIS_STICK_LEFT_LEFTWARDS];

		//double twist_angular_z_vel = _angular_speed_scale * joy->axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS];
		double twist_angular_z_vel = (32767.0 + joy->axes[4]) * _angular_speed_scale - (32767.0 + joy->axes[5]) * _angular_speed_scale;
	 
		twist.linear.x = twist_linear_x_vel;
        twist.linear.y = twist_linear_y_vel;
		twist.angular.z = twist_angular_z_vel;

		if (twist_linear_x_vel == 0 && twist_angular_z_vel == 0) {
			if (_last_zero_twist == false) {
				_cmd_vel_pub.publish(twist);
				_last_zero_twist = true;
				} 
		} else {
			_last_zero_twist = false;
			_cmd_vel_pub.publish(twist);
		}
}

#endif 