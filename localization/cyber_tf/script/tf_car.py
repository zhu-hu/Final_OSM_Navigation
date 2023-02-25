#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped


class CarTransformSystem:
    def __init__(self):
        rospy.init_node('tf_broadcaster')
        rospy.Subscriber('/current_pose', PoseStamped, self.handle_car_pose)
        self.br = tf.TransformBroadcaster()
        print("tf start!")

    def handle_car_pose(self, msg):
        # msg = PoseStamped()
        print("TF is working!")
        self.br.sendTransform((msg.pose.position.x,
                               msg.pose.position.y,
                               0),
                              (msg.pose.orientation.x,
                               msg.pose.orientation.y,
                               msg.pose.orientation.z,
                               msg.pose.orientation.w),
                              rospy.Time.now(),
                              "car",
                              "world")


if __name__ == '__main__':
    CarTransformSystem()
    rospy.spin()
