#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

#include "packet.h"
#include "imu_data_decode.h"

/*
 * @brief Open serial port with the given device name
 *
 * @return The file descriptor on success or -1 on error.
 */

/* recv freq */
static int frame_rate;

static uint8_t buf[2048];
void dump_data_packet(receive_imusol_packet_t *data);
	
int open_port(char *port_device)
{
	struct termios options;

	int fd = open(port_device, O_RDWR | O_NOCTTY);
	
	tcgetattr(fd, &options);

	if (fd == -1)
	{
		perror("open_port: Unable to open SerialPort");
		puts("Please check the usb port name!!!");
		puts("such as \" sudo ./main ttyUSB0 \"");
		exit(0);
	}

	if(fcntl(fd, F_SETFL, 0)<0)
	{
		printf("fcntl failed\n");
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
	}
  
	if(isatty(STDIN_FILENO)==0)
	{
		printf("standard input is not a terminal device\n");
	}
	else 
	{
		printf("isatty success!\n");
	}


	bzero(&options,sizeof(options));

	options.c_cflag = B115200 | CS8 | CLOCAL |CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options);
	return (fd);
}

void timer(int sig)
{
	if(SIGALRM == sig)
	{
		frame_rate = frame_count;
		frame_count = 0;
		alarm(1);
	}
}


int main(int argc, const char *argv[])
{
	int fd = 0;
	char dir_usb_dev[64] = "/dev/";

	int i;
	ssize_t n = 0;

	signal(SIGALRM, timer);
	if(argc >1)
	{
		strcat(dir_usb_dev, argv[1]);
		fd = open_port(dir_usb_dev);
	}
	else
	{
		puts("Please enter USB port append to the execution command!!!");
		exit(0);	
	}

	alarm(1);
	imu_data_decode_init(); 
				
	while(true)
	{
		n = read(fd, buf, sizeof(buf));

		if(n > 0)
		{
			for(i=0; i < n; i++)
			{
				packet_decode(buf[i]);
			}
			puts("\033c");

			if(receive_gwsol.tag != KItemGWSOL)
			{
			//	 printf imu data packet 
				dump_data_packet(&receive_imusol);

				puts("Please enter ctrl + 'c' to quit...");
			}
			else
			{
			//	 printf gw data packet 
				printf("        GW ID:  %-8d\n",receive_gwsol.gw_id);
				for(i = 0; i < receive_gwsol.n; i++)
				{ 
					dump_data_packet(&receive_gwsol.receive_imusol[i]);
					puts("");
				}
				
				puts("Please enter ctrl + 'c' to quit...");
			}
		}
	}	
	sleep(1);

	close(fd);
}

void dump_data_packet(receive_imusol_packet_t *data)
{
	if(bitmap & BIT_VALID_ID)
		printf("    Device ID:  %-8d\n",  data->id);

	if(bitmap & BIT_VALID_TIMES)
		printf("Run times(ms):  %d days  %d:%d:%d:%d \n",data->times / 86400000, data->times / 3600000 % 24, data->times / 60000 % 60, data->times / 1000 % 60, data->times % 1000);

	printf("   Frame Rate: %4dHz\n", frame_rate);

	if(bitmap & BIT_VALID_ACC)
		printf("       Acc(G):	%8.3f %8.3f %8.3f\r\n",  data->acc[0],  data->acc[1],  data->acc[2]);

	if(bitmap & BIT_VALID_GYR)
		printf("   gyr(deg/s):	%8.2f %8.2f %8.2f\r\n",  data->gyr[0],  data->gyr[1],  data->gyr[2]);

	if(bitmap & BIT_VALID_MAG)
		printf("      mag(uT):	%8.2f %8.2f %8.2f\r\n",  data->mag[0],  data->mag[1],  data->mag[2]);

	if(bitmap & BIT_VALID_EUL)
		printf("   eul(R P Y):  %8.2f %8.2f %8.2f\r\n",  data->eul[0],  data->eul[1],  data->eul[2]);

	if(bitmap & BIT_VALID_QUAT)
		printf("quat(W X Y Z):  %8.3f %8.3f %8.3f %8.3f\r\n",  data->quat[0],  data->quat[1],  data->quat[2],  data->quat[3]);
}
			
