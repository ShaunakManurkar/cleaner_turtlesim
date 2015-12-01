#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;

const double PI = 3.1415926535897;

ros::Publisher vel_pub;
ros::Subscriber pose_sub;
turtlesim::Pose bot_pose;

void poseCallBack(const turtlesim::Pose::ConstPtr& pose);
void move(double speed,double distance,bool isForward);
void rotate(double angular_speed,double angle, bool isClockwise);
double degreesToRadians(double degree_angle);
void desiredAngle(double radian_angle);
void cleanerRobot();
void spiralCleaner();

int main(int argc, char **argv)
{
	ros::init(argc,argv,"move_rotate_robot");
	ros::NodeHandle n;

	vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
	pose_sub = n.subscribe("/turtle1/pose",10,poseCallBack);

	/*move(1.0,4.0,1);
	ros::Rate loop_rate(0.5);
	loop_rate.sleep();

	desiredAngle(degreesToRadians(89));
	loop_rate.sleep();

	desiredAngle(degreesToRadians(bot_pose.theta+89));
	loop_rate.sleep();

	move(1.0,3.0,1);
	loop_rate.sleep();*/

	//cleanerRobot();
	spiralCleaner();
	ros::spin();
	//rotate(degreesToRadians(3.0), degreesToRadians(82), 0);
}

void spiralCleaner()
{
	geometry_msgs::Twist vel;

	double const_angular = 4;
	double rk = 0;
	ros::Rate loop_rate(10);

	while(bot_pose.x<10.5 && bot_pose.y<10.5)
	{
		rk += 0.4;
		vel.linear.x = rk;
		vel.linear.y = 0;
		vel.linear.z = 0;
		vel.angular.x = 0;
		vel.angular.y = 0;
		vel.angular.z = const_angular;
		vel_pub.publish(vel);
		loop_rate.sleep();
		ros::spinOnce();
	}

	vel.linear.x = 0;
	vel_pub.publish(vel);
}
void cleanerRobot()
{
	move(3.0,5.0,1);
	ros::Rate loop_rate(0.5);
	loop_rate.sleep();

	rotate(degreesToRadians(82),degreesToRadians(82),0);
	loop_rate.sleep();

	move(3.0,5.0,1);
	loop_rate.sleep();

	rotate(degreesToRadians(82),degreesToRadians(82),0);
	loop_rate.sleep();

	move(3.0,9.0,1);
	loop_rate.sleep();	

	rotate(degreesToRadians(82),degreesToRadians(82),0);
	loop_rate.sleep();

	move(3.0,9.0,1);
	loop_rate.sleep();

	rotate(degreesToRadians(82),degreesToRadians(82),0);
	loop_rate.sleep();

	move(3.0,8.0,1);
	loop_rate.sleep();

	rotate(degreesToRadians(82),degreesToRadians(82),0);
	loop_rate.sleep();	
}

void poseCallBack(const turtlesim::Pose::ConstPtr& pose)
{
	bot_pose.x = pose->x;
	bot_pose.y = pose->y;
	bot_pose.theta = pose->theta;
}

void rotate(double angular_speed, double angle, bool isClockwise)
{
	geometry_msgs::Twist vel_msg;

	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	if(isClockwise)
		vel_msg.angular.z = -abs(angle);
	else
		vel_msg.angular.z = abs(angle);

	double t0 = ros::Time::now().toSec();
	double curr_angle = 0;
	ros::Rate loop_rate(10);

	while(curr_angle <= angle)
	{
		vel_pub.publish(vel_msg);

		double t1 = ros::Time::now().toSec();
		curr_angle = angular_speed * (t1-t0);
		cout<<curr_angle<<endl;

		ros::spinOnce();
		loop_rate.sleep();
	}

	vel_msg.angular.z = 0;
	vel_pub.publish(vel_msg);
}

void desiredAngle(double radian_angle)
{
	double relative_angle = radian_angle - bot_pose.theta;
	//cout<<"relative angle is "<<relative_angle<<endl;
	bool isClockwise = ((relative_angle<=0)?true:false);
	rotate(abs(relative_angle),abs(relative_angle),isClockwise);
}

double degreesToRadians(double degree_angle)
{
	return (PI * degree_angle / 180);
}

void move(double speed, double distance, bool isForward)
{
	geometry_msgs::Twist vel;

	if(isForward)
		vel.linear.x = abs(speed);
	else
		vel.linear.x = -abs(speed);

	vel.linear.y = 0;
	vel.linear.z = 0;
	vel.angular.x = 0;
	vel.angular.y = 0;
	vel.angular.z = 0;

	double curr_dist = 0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);

	while(curr_dist <= distance)
	{
		vel_pub.publish(vel);
		double t1 = ros::Time::now().toSec();
		curr_dist = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}

	vel.linear.x = 0;
	vel_pub.publish(vel);
}