#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>

tf::TransformBroadcaster* br_ = nullptr;

///world->base_link
void PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg){
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, 0.0));
    tf::Quaternion q;
    q.setX(msg->pose.orientation.x);
    q.setY(msg->pose.orientation.y);
    q.setZ(msg->pose.orientation.z);
    q.setW(msg->pose.orientation.w);
    transform.setRotation(q);
    br_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}



int main(int argc, char** argv){
    ros::init(argc, argv, "tiggo_tf_broadcaster");
    ros::NodeHandle nh;
    br_ = new tf::TransformBroadcaster();
    ros::Subscriber sub_pose = nh.subscribe("current_pose", 1, &PoseCallback);

    std::cout << "Transform running!" << std::endl;
    std::cout<<"TF: world->base_link!!!"<<std::endl;

    ros::spin();
}
