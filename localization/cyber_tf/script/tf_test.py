#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PointStamped


class TFTest(object):
    def __init__(self):
        rospy.init_node("tf_test")
        self.listener = tf.TransformListener()

        self.pt_car = PointStamped()
        self.pt_car.header.frame_id = '/base_link'
        self.pt_car.point.x = 2.0
        self.pt_car.point.y = 1.0

        self.pt_world = None

        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            self.pt_world = self.listener.transformPoint('/world', self.pt_car)
            print("x: " + str(self.pt_world.point.x))
            print("y: " + str(self.pt_world.point.y))
            self.pt_car.point.y += 1;


if __name__ == "__main__":
    TFTest()
