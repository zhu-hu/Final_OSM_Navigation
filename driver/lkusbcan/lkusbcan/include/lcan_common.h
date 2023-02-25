#ifndef _LCAN_COMMON_H
#define	_LCAN_COMMON_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <iostream>

//gl8 can frame
#include "cyber_msgs/CanFrame.h"


//----------与can驱动相关的头文件及宏定义--------------------
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>

#include "ICANCmd.h"

//-----------------------------------------------------------


using namespace std;


#endif
