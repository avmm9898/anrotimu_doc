//imu subscriiber.cpp

#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <sensor_msgs/Imu.h>

#define ROSTOPIC_ECHO 1

void imu_callback(const sensor_msgs::Imu msg);

int main(int argc,char **argv)
{
	ros::init(argc,argv,"imu_subscriber");

	ros::NodeHandle n;
	
#if ROSTOPIC_ECHO
	execlp("rostopic", "rostopic", "echo", "/IMU_data", NULL);
#endif
	ros::Subscriber imu_sub = n.subscribe("/IMU_data", 10,imu_callback);
	
	ros::spin();
}

void imu_callback(const sensor_msgs::Imu msg)
{
	printf("\033c");

	printf("Quat(W X Y Z):  %8.3f	%8.3f	%8.3f	%8.3f\r\n", msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);

	printf("   Gyr(deg/s):			%8.3f	%8.3f	%8.3f\r\n", msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);

	printf("       Acc(G):			%8.3f	%8.3f	%8.3f\r\n", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);

}
