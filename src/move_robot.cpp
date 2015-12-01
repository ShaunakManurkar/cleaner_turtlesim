// This program shows the use of publisher and subsriber nodes to move turtlesim in a straight line.
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

using namespace std;

// Publisher to publish the msg.
ros::Publisher vel_pub;

// Function to move the robot in straight line.
void move(double speed, double distance, bool isForward);

int main(int argc, char **argv)
{
	// Initiate new ROS node named "move_robot"
	ros::init(argc,argv,"move_robot");
	// Create a node handler to create a reference for the node
	ros::NodeHandle n;

	// Initialize publihser which advertise the msg of given <type> on the given ("topic")
	vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);

	double speed, distance;
	bool isForward;

	cout<<"Enter the Speed: ";
	cin>>speed;
	cout<<"Enter the Distance: ";
	cin>>distance;
	cout<<"Moving Forward(1/0): ";
	cin>>isForward;
	move(speed,distance,isForward);
}

void move(double speed, double distance, bool isForward)
{
	// This is the type of message we want to publish on the tustlesim.
	geometry_msgs::Twist vel_msg;
	// Speed = distance / time

	// Set random motion in random direction
	if(isForward)
		vel_msg.linear.x = abs(speed);
	else
		vel_msg.linear.x = -abs(speed);
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	// Set angular velocity to 0
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	// t0: current time
	double t0 = ros::Time::now().toSec();
	double curr_distance = 0;

	// Msg will be pub;ished 10 times in one sec
	ros::Rate loop_rate(10);

	while(curr_distance<distance)
	{
		// Publish the msg to the topic cmd_vel
		vel_pub.publish(vel_msg);
		// Calculate the curr_distance
		double t1 = ros::Time::now().toSec();
		curr_distance = speed * (t1-t0);

		ros::spinOnce(); // Used to publish to the topic.
		loop_rate.sleep();	
	}

	vel_msg.linear.x = 0;
	vel_pub.publish(vel_msg);
}