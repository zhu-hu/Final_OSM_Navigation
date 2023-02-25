#ifndef CMD2E100CAN_H
#define CMD2E100CAN_H

#include "ros/ros.h"
// #include "e100_msgs/enablecmd.h"
// #include "e100_msgs/xboxone.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32MultiArray.h"
#include "cyber_msgs/CanFrame.h"
#include "cyber_msgs/VehicleSpeedFeedback.h"
#include "cyber_msgs/CommunicationHeart.h"
// #include "e100_msgs/brakecmd.h"

class cmd2e100can
{
public:
	cmd2e100can();
	void steercmdcallback(std_msgs::Float64 msg);
	void speedcmdcallback(std_msgs::Int32MultiArray msg);
	void msg_init(void);

	void canid_0x202_timer_callback(const ros::TimerEvent &);
private:
	bool auto_speed;
	bool auto_steer;
	ros::NodeHandle nh_;
	ros::Subscriber sub_steercmd_;
	ros::Subscriber sub_speedcmd_;
		ros::Publisher pub_canWrite_0x202_;

	ros::Timer canid_0x202_timer;



	ros::Publisher pub_canTest_;
};

#endif