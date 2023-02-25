#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cyber_msgs/JY901.h"

#include <sstream>

#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define RS_DEVICE "/dev/ttyUSB0"
#define BAUDRATE B115200
#define HEAD1 0x55
#define HEAD2 0x52
#define HEAD3 0x53
#define PI 3.1415926535
std::string port_name;

float yaw_rate = 0.0;
float pitch_rate = 0.0;
float roll_rate = 0.0;

float yaw_angle = 0.0;
float pitch_angle = 0.0;
float roll_angle = 0.0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jyimu");

  ros::NodeHandle n;
  ros::NodeHandle pnh("~");
  pnh.param("imu_port_name", port_name, std::string("/dev/ttyUSB0"));
  const char* device = port_name.data();

  ros::Rate loop_rate(100);  

  ros::Publisher jyimu_pub = n.advertise<cyber_msgs::JY901>("mti1_msg", 1000);
  cyber_msgs::JY901 jyimu_;

  int fd,c=0,res;
  struct termios oldtio,newtio;
  fd = open(device,O_RDWR|O_NOCTTY);
  if(fd<0)
  {
      perror("error");
      exit(1);
  }

  tcgetattr(fd,&oldtio);
  bzero(&newtio,sizeof(newtio));

  newtio.c_cflag = BAUDRATE|CS8|CLOCAL|CREAD;
  newtio.c_iflag |= IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;

  newtio.c_cc[VMIN]=0;
  newtio.c_cc[VTIME]=0;
  tcflush(fd,TCIFLUSH);
  tcsetattr(fd,TCSANOW,&newtio);
  
  unsigned char read_buf[200]={0};

  int count = 0;
  int wrong = 0;

  while (ros::ok())
  {
    //-----------------RX--------------------//
    unsigned char checksum1 = 0;
    unsigned char checksum2 = 0;
    int checkres = 0;
    res = read(fd,read_buf,22);
    if(res == 22 && read_buf[0]==HEAD1 && read_buf[11]==HEAD1 && read_buf[1]==HEAD2 && read_buf[12]==HEAD3)
    {
    	for(int i=0;i<10;++i)
    	{
    		checksum1 += read_buf[i];
    	}
    	for(int i=11;i<21;++i)
    	{
    		checksum2 += read_buf[i];
    	}
    	if (checksum1 == read_buf[10] && checksum2 == read_buf[21])
    	{
    		checkres = 1;

    		short temp = (read_buf[3]<<8) | read_buf[2];
    		roll_rate = temp/32768.0*2000.0 / 180.0 * PI;
    		temp = (read_buf[5]<<8) | read_buf[4];
    		pitch_rate = temp/32768.0*2000.0 / 180.0 * PI;
    		temp = (read_buf[7]<<8) | read_buf[6];
    		yaw_rate = temp/32768.0*2000.0 / 180.0 * PI;
    		temp = (read_buf[14]<<8) | read_buf[13];
    		roll_angle = temp/32768.0 * 180.0;
    		temp = (read_buf[16]<<8) | read_buf[15];
    		pitch_angle = temp/32768.0 * 180.0;
    		temp = (read_buf[18]<<8) | read_buf[17];
    		yaw_angle = temp/32768.0 * 180.0;

	        jyimu_.header.stamp = ros::Time::now();
	        jyimu_.header.frame_id = "jyimu";

	        jyimu_.gyro_x = roll_rate;
	        jyimu_.gyro_y = pitch_rate;
	        jyimu_.gyro_z = yaw_rate;

	        jyimu_.roll = roll_angle;
	        jyimu_.pitch = pitch_angle;
	        jyimu_.yaw = yaw_angle;

	        jyimu_pub.publish(jyimu_);
    	}
    	else 
        {
            checkres = 0;
        }
    }
    else 
    {
        res = read(fd,read_buf,22);
        wrong ++ ;
    }

    if(count == 50) 
    {
    	//ROS_INFO("data num %d ", res);
    	//ROS_INFO("check %d ", checkres);
    	//ROS_INFO("%x %x %x %x %x %x %x %x %x ",read_buf[0],read_buf[1],read_buf[2],read_buf[3],read_buf[4],read_buf[5],read_buf[6],read_buf[7],read_buf[8]);
    	//ROS_INFO("%x %x %x %x %x %x %x %x %x ",read_buf[9],read_buf[10],read_buf[11],read_buf[12],read_buf[13],read_buf[14],read_buf[15],read_buf[16],read_buf[17]);
    	//ROS_INFO("%x %x %x %x %x %x %x %x %x ",read_buf[18],read_buf[19],read_buf[20],read_buf[21],read_buf[22],read_buf[23],read_buf[24],read_buf[25],read_buf[26]);
    	//ROS_INFO("%x %x %x %x %x %x %x %x %x ",read_buf[27],read_buf[28],read_buf[29],read_buf[30],read_buf[31],read_buf[32],read_buf[33],read_buf[34],read_buf[35]);
    	//ROS_INFO("%x %x %x %x %x %x %x %x %x ",read_buf[36],read_buf[37],read_buf[38],read_buf[39],read_buf[40],read_buf[41],read_buf[42],read_buf[43],read_buf[44]);
    	//ROS_INFO("%x %x %x %x %x ",read_buf[45],read_buf[46],read_buf[47],read_buf[48],read_buf[49]);
    	//ROS_INFO("yaw rate %f ", yaw_rate);
    	//ROS_INFO("pitch rate %f ", pitch_rate);
    	//ROS_INFO("roll rate %f ", roll_rate);
    	ROS_INFO("anglesdeg %f  %f  %f ",roll_angle,pitch_angle,yaw_angle);
      ROS_INFO("wrong frames: %d ", wrong);
        
      count = 0;
    }

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  tcsetattr(fd,TCSANOW,&oldtio);
  close(fd);

  return 0;
}
