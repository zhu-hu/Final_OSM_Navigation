
#include "ros/ros.h"
#include <string>
#include <iostream>
//#include <fstream>
//#include <strstream>
#include "cyber_msgs/CanFrame.h"
#include "cyber_msgs/VehicleSteerFeedback.h"
#include "cyber_msgs/VehicleSpeedFeedback.h"
#include "cyber_msgs/VehicleBrakeFeedback.h"
#include "cyber_msgs/CommunicationHeart.h"
#include "cyber_msgs/VehicleState.h"

using namespace std;
class CanFrameDecoder
{
public:
    CanFrameDecoder();
    void canFrameCallback(cyber_msgs::CanFrame can_msg);
    void printSteerInfo();
    void printSpeedInfo();
    void printImuInfo();
    void logSpeedInfo();

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_can_msg_;
    ros::Publisher pub_speed_;
    ros::Publisher pub_steer_;
    ros::Publisher pub_brake_;
    ros::Publisher pub_communicationHeart_;
    ros::Publisher pub_vehicle_state_;
    int id_;
    int len_;
    unsigned char data_[8];
    short a_, b_; //这里要注意，必须定义为整形变量，否则会丢失数据符号！
    float steer_angle_;
    float steer_rate_;
};

CanFrameDecoder::CanFrameDecoder()
{
    sub_can_msg_ = nh_.subscribe("/can_frame", 10, &CanFrameDecoder::canFrameCallback, this);
    pub_steer_ = nh_.advertise<cyber_msgs::VehicleSteerFeedback>("/e100/steer_feedback", 1);
    pub_speed_ = nh_.advertise<cyber_msgs::VehicleSpeedFeedback>("/e100/speed_feedback", 1);
    pub_brake_ = nh_.advertise<cyber_msgs::VehicleBrakeFeedback>("/e100/brake_feedback", 1);
    pub_vehicle_state_ = nh_.advertise<cyber_msgs::VehicleState>("/e100/vehicle_state", 1);
    pub_communicationHeart_ = nh_.advertise<cyber_msgs::CommunicationHeart>("/e100/communicationHeart", 1);
    ros::NodeHandle nh_priv_("~");
    ros::spin();
}

/*void CanFrameDecoder::printSpeedInfo()
{
    cout<<"frame_id: "<<std::hex<<id_<<"\t";
    cout<<"data_len: "<<std::dec<<len_<<"\t"
        <<"front_wheel_speed: "<<front_wheel_speed_<<"\t"
        <<"rear_wheel_speed: "<<rear_wheel_speed_<<"\t"
        <<"front_left_pulse: "<<front_left_pulse_<<"\t"
        <<"front_right_pulse: "<<front_right_pulse_<<"\t"
        <<"rear_left_pulse: "<<rear_left_pulse_<<"\t"
        <<"rear_right_pulse: "<<rear_right_pulse_<<endl;
}

void CanFrameDecoder::logSpeedInfo()
{
	cout<<"can data: ";
	for(int i=0; i<8; i++)
		cout<<data_[i]<<" ";
	cout<<endl;
	cout<<"frame_id: "<<std::hex<<id_<<"\t";
    cout<<"data_len: "<<std::dec<<len_<<"\t"
        <<"front_wheel_speed: "<<front_wheel_speed_<<"\t"
        <<"rear_wheel_speed: "<<rear_wheel_speed_<<"\t"
        <<"front_left_pulse: "<<front_left_pulse_<<"\t"
        <<"front_right_pulse: "<<front_right_pulse_<<"\t"
        <<"rear_left_pulse: "<<rear_left_pulse_<<"\t"
        <<"rear_right_pulse: "<<rear_right_pulse_<<endl;
}

void CanFrameDecoder::printSteerInfo()
{
    cout<<"frame_id: "<<std::hex<<id_<<"\t";
    cout<<"data_len: "<<std::dec<<len_<<"\t"
        <<"steer_angle: "<<steer_angle_<<"\t"
        <<"steer_rate: "<<steer_rate_<<endl;
}

void CanFrameDecoder::printImuInfo()
{
    cout<<"frame_id: "<<std::hex<<id_<<"\t";
    cout<<"data_len: "<<std::dec<<len_<<"\t"
        <<"acc_x: "<<acc_x_<<"\t"
        <<"acc_y: "<<acc_y_<<"\t"
        <<"yaw_z: "<<yaw_z_<<endl;
}
*/

cyber_msgs::VehicleSpeedFeedback speed_msg;
cyber_msgs::VehicleSteerFeedback steer_msg;
cyber_msgs::VehicleBrakeFeedback brake_msg;
cyber_msgs::CommunicationHeart communication_msg;
cyber_msgs::VehicleState vehicleState_msg;
void CanFrameDecoder::canFrameCallback(cyber_msgs::CanFrame can_msg)
{
    id_ = can_msg.id;
    len_ = can_msg.len;
    float speed_cmps_;
    float speed_kmph_;
    float steer_0p1d_;
    for (int i = 0; i < len_; i++)
        data_[i] = can_msg.data[i];

    switch (id_)
    {

    case 0x182:
        speed_cmps_ = data_[2] + data_[3] * 256;
        speed_msg.is_updated = true;
        //speed_msg.speed_kmph = int(speed_kmph_);
        speed_msg.speed_cmps = int(speed_cmps_);
        pub_speed_.publish(speed_msg);

        steer_0p1d_ = data_[0] + data_[1] * 256;
        if (steer_0p1d_ > 30000)
        {
            steer_0p1d_ = steer_0p1d_ - 65535;
        }
        steer_msg.steer_0p1d = steer_0p1d_;
        //steer_msg.steer_torque = data_[3] * 0.1;
        //steer_msg.steer_fault = data_[4] >> 6;
        pub_steer_.publish(steer_msg);

        break;
    case 0x282:

        vehicleState_msg.emergencyIsEnabled = data_[0] & 0x01;
        vehicleState_msg.autoCanBeEnabled = data_[0] >> 1 & 0x01;
        vehicleState_msg.beAutoState = data_[0] >> 2 & 0x01;
        vehicleState_msg.vehicleCheckedIsOk = data_[0] >> 4 & 0x01;
        vehicleState_msg.taskEnabled = data_[6] & 0x01;
        pub_vehicle_state_.publish(vehicleState_msg);
        break;
    case 0x302: //speed
        speed_msg.drive_mode = data_[0] >> 7 & 0x01;
        speed_msg.drive_shift = data_[0] >> 5 & 0x03;
        speed_msg.brake_enable = data_[0] >> 3 & 0x03;
        speed_msg.key_state = data_[0] >> 1 & 0x03;
        speed_msg.brakePedal_position = data_[0] >> 0 & 0x01;

        speed_kmph_ = (data_[1] * 256 + data_[2]) * 0.015625;
        if (speed_kmph_ > 511.98438)
        {
            speed_kmph_ = speed_kmph_ - 1024;
        }

        speed_msg.acc = data_[3];
        speed_msg.brake_re = data_[4];
        speed_msg.motor_fault = data_[6] & 0x15;

        speed_cmps_ = speed_kmph_ * 1000 / 36;
        speed_msg.header.stamp = can_msg.header.stamp;
        speed_msg.is_updated = true;
        speed_msg.speed_kmph = int(speed_kmph_);
        speed_msg.speed_cmps = int(speed_cmps_);
        pub_speed_.publish(speed_msg);
        break;

    case 0x704: //steer

        steer_msg.header.stamp = can_msg.header.stamp;
        steer_msg.steer_mode = data_[0];
        steer_0p1d_ = (data_[1] * 256 + data_[2]) * 0.1 - 740;
        steer_msg.steer_0p1d = steer_0p1d_;
        steer_msg.steer_torque = data_[3] * 0.1;
        steer_msg.steer_fault = data_[4] >> 6;
        pub_steer_.publish(steer_msg);
        break;

    case 0x132: //brake

        brake_msg.header.stamp = can_msg.header.stamp;
        brake_msg.ebsState = (data_[0] >> 3) & 0x01;
        brake_msg.ebsMode = (data_[0] >> 2) & 0x01;
        brake_msg.iduReqOk = (data_[2] >> 3) & 0x01;
        brake_msg.ebsPress = data_[3];
        brake_msg.brakePedalPosition = data_[4] * 256 + data_[5];
        pub_brake_.publish(brake_msg);

        communication_msg.brakeCommunicationIson = 1;
        pub_communicationHeart_.publish(communication_msg);
        break;

    default:
        break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_decoder_node");
    CanFrameDecoder DecoderObject;

    return 0;
}
