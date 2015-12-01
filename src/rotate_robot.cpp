// Rotate the cleaner robot to the desired degrees.
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;

const double PI = 3.1415;

ros::Publisher vel_pub;
ros::Subscriber pose_sub;
turtlesim::Pose turtlesim_pose;

void rotate(double angular_speed, double angle, bool isClockwise);
double degrees_to_radian(double degree_angle);
void desired_orientation(double radian_angle);
void poseCallBack(const turtlesim::Pose::ConstPtr & pose_msg);

int main(int argc,char **argv)
{
	// Initialize the node
	ros::init(argc, argv, "rotate_robot");
	// Initiate a node handler
	ros::NodeHandle n;

	vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);

	pose_sub = n.subscribe("/turtle1/pose",10,poseCallBack);

	double angular_speed,degree_angle,radian_angle;
	bool isClockwise;

	/*cout<<"Enter angular speed: ";
	cin>>angular_speed;
	cout<<"Clockwise(1/0): ";
	cin>>isClockwise;*/

	//cout<<"Enter desired angle: ";
	//cin>>degree_angle;

	desired_orientation(degrees_to_radian(60));

	ros::Rate loop_rate(0.5);
	loop_rate.sleep();

	desired_orientation(degrees_to_radian(120));

	//ros::Rate loop_rate(10);
	loop_rate.sleep();
	
	desired_orientation(degrees_to_radian(0));

	//ros::Rate loop_rate(0.5);
	loop_rate.sleep();
	
	ros::spin();
	//rotate(angular_speed,radian_angle,isClockwise);
}

void rotate(double angular_speed, double angle, bool isClockwise)
{
	geometry_msgs::Twist vel_msgs;

	// Initialize all non-require component to zero.
	vel_msgs.linear.x = 0;
	vel_msgs.linear.y = 0;
	vel_msgs.linear.z = 0;

	vel_msgs.angular.x = 0;
	vel_msgs.angular.y = 0;

	if(isClockwise)
		vel_msgs.angular.z = -abs(angle);
	else
		vel_msgs.angular.z = abs(angle);

	double curr_angle = 0;
	double t0 = ros::Time::now().toSec();

	ros::Rate loop_rate(10);

	while(curr_angle <= angle)
	{
		vel_pub.publish(vel_msgs);

		double t1 = ros::Time::now().toSec();
		curr_angle = angle * (t1-t0);

		ros::spinOnce();
		loop_rate.sleep();
	}

	vel_msgs.angular.z = 0;
	vel_pub.publish(vel_msgs);
}

double degrees_to_radian(double degree_angle)
{
	return (degree_angle * PI / 180.0);
}

void poseCallBack(const turtlesim::Pose::ConstPtr & pose_msg)
{
	turtlesim_pose.x = pose_msg->x;
	turtlesim_pose.y = pose_msg->y;
	turtlesim_pose.theta = pose_msg->theta;
	//cout<<turtlesim_pose.theta<<endl;
}

void desired_orientation(double radian_angle)
{
	cout<<turtlesim_pose.theta<<endl;
	double relative_angle = radian_angle - turtlesim_pose.theta;
	bool isClockwise = ((relative_angle<=0)?true:false);
	rotate(abs(relative_angle),abs(relative_angle),isClockwise);
}