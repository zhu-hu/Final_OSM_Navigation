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
#define HEAD 0xFA
#define PI 3.1415926535
std::string port_name;

float yaw_rate = 0.0;
float pitch_rate = 0.0;
float roll_rate = 0.0;
float accx = 0.0;
float accy = 0.0;
float accz = 0.0;
float magx = 0.0;
float magy = 0.0;
float magz = 0.0;

//IMU GLOBAL
float exInt, eyInt, ezInt; 
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float mygetqval[9];
float gx, gy, gz, ax, ay, az, mx, my, mz; 
float q[4];
float angle[3] = {0};
float yaw_temp,pitch_temp,roll_temp;
float last_yaw_temp,last_pitch_temp,last_roll_temp;
float yaw_angle,pitch_angle,roll_angle;

/*
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);  //have problem in ubuntu
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
*/

//#define BOARD_DOWN 1   //board lay up
bool Quaternion_Inited = 0;
void Init_Quaternion()
{
	float hx,hy,hz;
	hx = magx;hy=magy;hz=magz;
	#ifdef BOARD_DOWN
	if(hx<0 && hy <0)   //OK
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
		
	}
	else if (hx<0 && hy > 0) //OK
	{
		if(fabs(hx/hy)>=1)   
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
		
	}
	else if (hx > 0 && hy > 0)   //OK
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
		
	}
	else if (hx > 0 && hy < 0)     //OK
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;			
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}		
	}
	#else
		if(hx<0 && hy <0)
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
		
	}
	else if (hx<0 && hy > 0)
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
		
	}
	else if (hx>0 && hy > 0)
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;			
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}		
	}
	#endif
}

void IMU_getValues() {  
	mygetqval[0] = accx;
	mygetqval[1] = accy;
	mygetqval[2] = accz;
	mygetqval[3] = roll_rate;
	mygetqval[4] = pitch_rate;
	mygetqval[5] = yaw_rate;
	mygetqval[6] = magx;
	mygetqval[7] = magy;
	mygetqval[8] = magz;
}

double dt = 0;
double time_now = 0;
double time_last = 0;
#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases
void IMU_AHRSupdate(void) {
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez,halfT;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;   

    gx = mygetqval[3];
    gy = mygetqval[4];
    gz = mygetqval[5];
    ax = mygetqval[0];
    ay = mygetqval[1];
    az = mygetqval[2];
    mx = mygetqval[6];
    my = mygetqval[7];
    mz = mygetqval[8];		

    halfT = 0.005;   
    
    norm = 1.0/sqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    
    norm = 1.0/sqrt(mx*mx + my*my + mz*mz);          
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm; 
    // compute reference direction of flux
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;	
        ezInt = ezInt + ez * Ki * halfT;
        
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
    
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

    
    norm = 1.0/sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;

}

void IMU_getQ(float * q) {

    IMU_getValues();	 
    IMU_AHRSupdate();
    q[0] = q0; 
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

void IMU_getYawPitchRoll(float * angles) 
{  
    IMU_getQ(q); 
    
    angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1) * 180.0 / PI;  // yaw        -pi----pi
    angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180.0 / PI; // pitch    -pi/2    --- pi/2 
    angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1) * 180.0 / PI;  // roll       -pi-----pi  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mti1");

  ros::NodeHandle n;
	ros::NodeHandle pnh("~");
	pnh.param("imu_port_name", port_name, std::string("/dev/ttyUSB0"));
  const char* device = port_name.data();

  ros::Rate loop_rate(50);  

  ros::Publisher mti1_pub = n.advertise<cyber_msgs::JY901>("mti1_msg", 1000);
  cyber_msgs::JY901 mti1_;

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

  int countcnt = 0;
  float yaw_sum = 0;
  float yaw_ave = 0;
  float pitch_sum = 0;
  float pitch_ave = 0;
  float roll_sum = 0;
  float roll_ave = 0;

  int count = 0;
  int wrong = 0;

  while (ros::ok())
  {
    //-----------------RX--------------------//
    unsigned int checksum = 0;
    int checkres = 0;
    res = read(fd,read_buf,20);
    if(res == 20 && read_buf[0]==HEAD)
    {
    	for(int i=1;i<res;++i)
    	{
    		checksum += read_buf[i];
    	}
    	if (checksum%256 == 0)
    	{
    		  checkres = 1;
	        unsigned char * temp = (unsigned char*)&yaw_rate;
	        for (int i = 0;i<4;i++){temp[i] = (unsigned char)read_buf[(3-i)+15];}
	        temp = (unsigned char*)&pitch_rate;
	        for (int i = 0;i<4;i++){temp[i] = (unsigned char)read_buf[(3-i)+11];}
    		  temp = (unsigned char*)&roll_rate;
	        for (int i = 0;i<4;i++){temp[i] = (unsigned char)read_buf[(3-i)+7];}

	        // temp = (unsigned char*)&accx;
	        // for (int i = 0;i<4;i++){temp[i] = (unsigned char)read_buf[(3-i)+7];}
	        // temp = (unsigned char*)&accy;
	        // for (int i = 0;i<4;i++){temp[i] = (unsigned char)read_buf[(3-i)+11];}
	        // temp = (unsigned char*)&accz;
	        // for (int i = 0;i<4;i++){temp[i] = (unsigned char)read_buf[(3-i)+15];}

	        // temp = (unsigned char*)&magx;
	        // for (int i = 0;i<4;i++){temp[i] = (unsigned char)read_buf[(3-i)+37];}
	        // temp = (unsigned char*)&magy;
	        // for (int i = 0;i<4;i++){temp[i] = (unsigned char)read_buf[(3-i)+41];}
	        // temp = (unsigned char*)&magz;
	        // for (int i = 0;i<4;i++){temp[i] = (unsigned char)read_buf[(3-i)+45];}

            //yaw_rate = yaw_rate - 0.0050; //ruihu
            //pitch_rate = pitch_rate - 0.0050;  //ruihu
            //roll_rate = roll_rate + 0.0009;  //ruihu
            /*
	        if(countcnt<1000)
	        {
	        	countcnt ++;
	        	yaw_sum += yaw_rate;
            pitch_sum += pitch_rate;
            roll_sum += roll_rate;
	        }
	        else if (countcnt == 1000)
	        {
	        	yaw_ave = yaw_sum/1000.0;  
            pitch_ave = pitch_sum/1000.0;  
            roll_ave = roll_sum/1000.0;   
	        	countcnt = 0;
	        	yaw_sum = 0;
            pitch_sum = 0;
            roll_sum = 0;
	        }
            */
          
	        // if(!Quaternion_Inited)
	        // {
	        // 	Quaternion_Inited = 1;
	        // 	Init_Quaternion();
	        // }
	        // else IMU_getYawPitchRoll(angle);

            mti1_.header.stamp = ros::Time::now();
            mti1_.header.frame_id = "mti1";

            mti1_.gyro_x = roll_rate;
            mti1_.gyro_y = pitch_rate;
            mti1_.gyro_z = yaw_rate;
 
            // mti1_.roll = angle[2];
            // mti1_.pitch = angle[1];
            // mti1_.yaw = angle[0];

            mti1_pub.publish(mti1_);
    	}
    	else 
        {
          checkres = 0;
        }
    }
    else 
    {
      res = read(fd,read_buf,20);
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
    	ROS_INFO("mti yaw %f ", yaw_rate);
    	//ROS_INFO("mti pitch %f ", pitch_rate);
    	//ROS_INFO("mti roll %f ", roll_rate);
    	// ROS_INFO("mti accx %f ", accx);
    	// ROS_INFO("mti accy %f ", accy);
    	// ROS_INFO("mti accz %f ", accz);
    	// ROS_INFO("mti magx %f ", magx);
    	// ROS_INFO("mti magy %f ", magy);
    	// ROS_INFO("mti magz %f ", magz);
    	// ROS_INFO("Q %f  %f  %f  %f ", q0,q1,q2,q3);
    	// ROS_INFO("anglesdeg %f  %f  %f ",angle[0],angle[1],angle[2]);
    	//ROS_INFO("YAW AVE %f ", yaw_ave);
      //ROS_INFO("PITCH AVE %f ", pitch_ave);
      //ROS_INFO("ROLL AVE %f ", roll_ave);
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
