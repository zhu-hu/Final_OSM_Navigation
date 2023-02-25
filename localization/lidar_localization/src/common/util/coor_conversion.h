#ifndef SRC_COOR_CONVERSION_H
#define SRC_COOR_CONVERSION_H

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include "common/struct/StateEstimation.h"
#include "common/struct/Pose2DStamped.h"
#include "common/struct/Point.h"
#include <geodesy/utm.h>
#include <sensor_msgs/Imu.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <sensor_msgs/NavSatFix.h>

namespace localization{
    namespace util{

        inline void FixmsgDecode(localization::StateEstimation *gps_posture,const sensor_msgs::NavSatFixConstPtr fix_msg,
                                 double cali_x,double cali_y,localization::Point utm_ori){
            gps_posture->timestamp = fix_msg->header.stamp.toSec();
            geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
            gps_msg->header = fix_msg->header;
            gps_msg->position.latitude = fix_msg->latitude;
            gps_msg->position.longitude = fix_msg->longitude;
            gps_msg->position.altitude = fix_msg->altitude;

            geodesy::UTMPoint utm_point;
            geodesy::fromMsg(gps_msg->position, utm_point);
            double temp_x,temp_y,temp_z;
            double temp_rx,temp_ry,temp_rz;
            temp_x = utm_point.easting - utm_ori.x;
            temp_y = utm_point.northing - utm_ori.y;
            temp_z=0;

            if(!std::isnan(gps_posture->pose.orientation.x)){
                tf::Matrix3x3(tf::Quaternion(gps_posture->pose.orientation.x,
                                         gps_posture->pose.orientation.y,
                                         gps_posture->pose.orientation.z,
                                         gps_posture->pose.orientation.w))
                        .getRPY(temp_rx,temp_ry,temp_rz);
                // temp_x += cali_x*cos(temp_rz)-cali_y*sin(temp_rz);
                // temp_y += cali_x*sin(temp_rz)+cali_y*cos(temp_rz);
                temp_x += cali_x * sin(temp_rz) + cali_y * cos(temp_rz);
                temp_y += -cali_x * cos(temp_rz) + cali_y * sin(temp_rz);
            }
            gps_posture->pose.position.x = temp_x;
            gps_posture->pose.position.y = temp_y;
            gps_posture->pose.position.z = temp_z;
        }

        inline nav_msgs::Odometry StaEstoOdom(localization::StateEstimation posture,std::string frame_id){
            nav_msgs::Odometry GpsResultMsg;
            GpsResultMsg.pose.pose.position.x = posture.pose.position.x;
            GpsResultMsg.pose.pose.position.y = posture.pose.position.y;
            GpsResultMsg.pose.pose.position.z = posture.pose.position.z;
            GpsResultMsg.pose.pose.orientation.x = posture.pose.orientation.x;
            GpsResultMsg.pose.pose.orientation.y = posture.pose.orientation.y;
            GpsResultMsg.pose.pose.orientation.z = posture.pose.orientation.z;
            GpsResultMsg.pose.pose.orientation.w = posture.pose.orientation.w;
            GpsResultMsg.header.frame_id = frame_id;
            GpsResultMsg.header.stamp = ros::Time().fromSec(posture.timestamp);
            return GpsResultMsg;
        }
        inline nav_msgs::Odometry LocaRestoOdom(const localization::Pose2DStamped *localization_result,std::string frame_id){
            nav_msgs::Odometry OutResultMsg;
            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,localization_result->pose.phi);
            OutResultMsg.pose.pose.position.x = localization_result->pose.x;
            OutResultMsg.pose.pose.position.y = localization_result->pose.y;
            OutResultMsg.pose.pose.position.z = 0;
            OutResultMsg.pose.pose.orientation.x = quat.x;
            OutResultMsg.pose.pose.orientation.y = quat.y;
            OutResultMsg.pose.pose.orientation.z = quat.z;
            OutResultMsg.pose.pose.orientation.w = quat.w;
            OutResultMsg.header.frame_id = frame_id;
            OutResultMsg.header.stamp = ros::Time().fromSec(localization_result->timestamp);
            return OutResultMsg;
        }
        inline nav_msgs::Odometry LocaRestoOdom(const localization::Pose poseture,std::string frame_id,ros::Time time){
            nav_msgs::Odometry OutResultMsg;
            OutResultMsg.pose.pose.position.x = poseture.position.x;
            OutResultMsg.pose.pose.position.y = poseture.position.y;
            OutResultMsg.pose.pose.position.z = poseture.position.z;
            OutResultMsg.pose.pose.orientation.x = poseture.orientation.x;
            OutResultMsg.pose.pose.orientation.y = poseture.orientation.y;
            OutResultMsg.pose.pose.orientation.z = poseture.orientation.z;
            OutResultMsg.pose.pose.orientation.w = poseture.orientation.w;
            OutResultMsg.header.frame_id = frame_id;
            OutResultMsg.header.stamp = time;
            return OutResultMsg;
        }
        inline void RPYtoStaEsOri(localization::StateEstimation *posture,double rx,double ry,double rz){
            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(rx,ry,rz);
            posture->pose.orientation.x = quat.x;
            posture->pose.orientation.y = quat.y;
            posture->pose.orientation.z = quat.z;
            posture->pose.orientation.w = quat.w;
        }
        inline void CalStaEsTwi(localization::StateEstimation *posture,double speed){
            double rx,ry,rz;
            tf::Matrix3x3(tf::Quaternion(posture->pose.orientation.x,
                                         posture->pose.orientation.y,
                                         posture->pose.orientation.z,
                                         posture->pose.orientation.w))
                                        .getRPY(rx,ry,rz);
            posture->twist.linear.x = speed * cos(rz);
            posture->twist.linear.y = speed * sin(rz);
            posture->twist.linear.z = 0;
        }
        inline void UpdateGlobalPosition(localization::StateEstimation *posture,const localization::Pose2DStamped *localization_result){
            
            posture->pose.position.x = localization_result->pose.x;
            posture->pose.position.y = localization_result->pose.y;
            posture->pose.position.z = 0;
            double temp_rx = 0.0, temp_ry = 0.0, temp_rz = localization_result->pose.phi;
            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(temp_rx,temp_ry,temp_rz);
            posture->pose.orientation.x = quat.x;
            posture->pose.orientation.y = quat.y;
            posture->pose.orientation.z = quat.z;
            posture->pose.orientation.w = quat.w;
        }
        inline sensor_msgs::NavSatFix LocaRestoNav(const localization::Pose2DStamped *localization_result,
                                                    localization::Point utm_ori,char utm_band,u_int8_t utm_zone){
            
            geodesy::UTMPoint utm_point;
            utm_point.band = utm_band;
            utm_point.zone = utm_zone;
            utm_point.easting = localization_result->pose.x + utm_ori.x;
            utm_point.northing = localization_result->pose.y + utm_ori.y;
            utm_point.altitude = 0;
            geographic_msgs::GeoPoint geo_point=geodesy::toMsg(utm_point);
            sensor_msgs::NavSatFix slam_fix_out;
            slam_fix_out.header.stamp = ros::Time().fromSec(localization_result->timestamp);
            slam_fix_out.longitude = geo_point.longitude;
            slam_fix_out.latitude = geo_point.latitude;
            slam_fix_out.altitude = geo_point.altitude;
            return slam_fix_out;
        }
        inline sensor_msgs::Imu LocaRestoImu(const localization::Pose2DStamped *localization_result){
            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,localization_result->pose.phi);
            sensor_msgs::Imu slam_heading_out;
            slam_heading_out.header.stamp = ros::Time().fromSec(localization_result->timestamp);
            slam_heading_out.orientation.x = quat.x;
            slam_heading_out.orientation.y = quat.y;
            slam_heading_out.orientation.z = quat.z;
            slam_heading_out.orientation.w = quat.w;
            return slam_heading_out;
        }
    }
}
#endif