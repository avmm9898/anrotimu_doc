//subscriiber 0x91 data package 

#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <serial_imu/Imu_0x91_msg.h>
#include "imu_data_decode.h"

void imu_0x91_callback(const serial_imu::Imu_0x91_msg imu_0x91_msg);

int main(int argc,char **argv)
{
	ros::init(argc,argv,"sub_0x91");

	ros::NodeHandle n;
	
	ros::Subscriber imu_0x91_sub = n.subscribe("/imu_0x91_package", 10, imu_0x91_callback);

	ros::spin();
}




void imu_0x91_callback(const serial_imu::Imu_0x91_msg imu_0x91_msg)
{
	printf("\033c");

	if(imu_0x91_msg.bitmap & BIT_VALID_ID)
		printf("     Devie ID:%6d\n",imu_0x91_msg.id);

	if(imu_0x91_msg.bitmap & BIT_VALID_TIMES)
		printf("    Run times: %d days  %d:%d:%d:%d\n",imu_0x91_msg.times / 86400000, imu_0x91_msg.times / 3600000 % 24, imu_0x91_msg.times / 60000 % 60, imu_0x91_msg.times / 1000 % 60, imu_0x91_msg.times % 1000);

	printf("  Frame Rate:  %4dHz\r\n", imu_0x91_msg.frame_rate);
	if(imu_0x91_msg.bitmap & BIT_VALID_ACC)
		printf("       Acc(G):%8.3f %8.3f %8.3f\r\n", imu_0x91_msg.acc_x, imu_0x91_msg.acc_y, imu_0x91_msg.acc_z);

	if(imu_0x91_msg.bitmap & BIT_VALID_GYR)
		printf("   Gyr(deg/s):%8.2f %8.2f %8.2f\r\n", imu_0x91_msg.gyr_x, imu_0x91_msg.gyr_y, imu_0x91_msg.gyr_z);

	if(imu_0x91_msg.bitmap & BIT_VALID_MAG)
		printf("      Mag(uT):%8.2f %8.2f %8.2f\r\n", imu_0x91_msg.mag_x, imu_0x91_msg.mag_y, imu_0x91_msg.mag_z);

	if(imu_0x91_msg.bitmap & BIT_VALID_EUL)
		printf("   Eul(R P Y):%8.2f %8.2f %8.2f\r\n", imu_0x91_msg.eul_r, imu_0x91_msg.eul_p, imu_0x91_msg.eul_y);

	if(imu_0x91_msg.bitmap & BIT_VALID_QUAT)
		printf("Quat(W X Y Z):%8.3f %8.3f %8.3f %8.3f\r\n", imu_0x91_msg.quat_w, imu_0x91_msg.quat_x, imu_0x91_msg.quat_y, imu_0x91_msg.quat_z);
}
