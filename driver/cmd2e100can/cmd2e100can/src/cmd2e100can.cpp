#include "cmd2e100can.h"

int16_t global_steer_0p1d_;
unsigned int global_speed_cmps_;
int16_t global_brake_dcc_;
cyber_msgs::CanFrame cmd_msg_0x202_;
cmd2e100can::cmd2e100can()
{
	auto_speed = false;
	auto_steer = false;
	ros::NodeHandle nh_priv("~");
	nh_priv.param<bool>("auto_speed", auto_speed, false);
	nh_priv.param<bool>("auto_steer", auto_steer, false);
	pub_canWrite_0x202_ = nh_.advertise<cyber_msgs::CanFrame>("/can_write_0x202", 50);
	canid_0x202_timer = nh_.createTimer(ros::Duration(0.01), &cmd2e100can::canid_0x202_timer_callback, this);

	if (auto_speed)
	{
		ROS_INFO("Enable auto speed!");

		sub_speedcmd_ = nh_.subscribe("/speed_cmd", 10, &cmd2e100can::speedcmdcallback, this); //speedcmd

	}
	else
		ROS_INFO("Disable auto speed!");

	if (auto_steer)
	{
		ROS_INFO("Enable auto steer!");
		sub_steercmd_ = nh_.subscribe("/steer_cmd", 10, &cmd2e100can::steercmdcallback, this);
		
	}
	else
		ROS_INFO("Disable auto steer!");
}

void cmd2e100can::canid_0x202_timer_callback(const ros::TimerEvent &)
{

	cmd_msg_0x202_.id = 0x202;
	cmd_msg_0x202_.len = 8;
	cmd_msg_0x202_.data[0] = global_steer_0p1d_;
	cmd_msg_0x202_.data[1] = global_steer_0p1d_/256;
	cmd_msg_0x202_.data[2] = global_speed_cmps_;
	cmd_msg_0x202_.data[3] = global_speed_cmps_/256;
	cmd_msg_0x202_.data[4] = global_brake_dcc_;
	cmd_msg_0x202_.data[5] =auto_steer;
	cmd_msg_0x202_.data[6] = auto_speed;
	cmd_msg_0x202_.data[7] = 0;
	pub_canWrite_0x202_.publish(cmd_msg_0x202_);
}


void msg_init(void)
{
	global_steer_0p1d_ = 0;
	global_speed_cmps_ = 0;
}


void cmd2e100can::steercmdcallback(std_msgs::Float64 msg)
{
	global_steer_0p1d_ =( int16_t)(msg.data);
}

void cmd2e100can::speedcmdcallback(std_msgs::Int32MultiArray msg)
{
	if(msg.data[0] < 0){
		global_speed_cmps_ = (unsigned int)(msg.data[0] + 65536);
	}else{
		global_speed_cmps_ = (unsigned int)msg.data[0];
	}
		std::cout<<"data:testData:" <<global_speed_cmps_<<std::endl;

	//global_speed_cmps_ = (int16_t)(msg.data[0]);
	global_brake_dcc_ = (int16_t)(msg.data[1]);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cmd2e100can");
	msg_init();
	cmd2e100can cmd2e100can_;
	ros::spin();
	return 0;
}
