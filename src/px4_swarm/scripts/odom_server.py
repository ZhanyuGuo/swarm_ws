#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry


class OdomServer:
    def __init__(self):
        rospy.init_node('odom_server', anonymous=True)

        self.init_x = rospy.get_param("~init_x")
        self.init_y = rospy.get_param("~init_y")
        self.init_z = rospy.get_param("~init_z")

        self.pub = rospy.Publisher('/odom_out', Odometry, queue_size=10)
        self.sub = rospy.Subscriber('/odom_in', Odometry, self.odom_callback)

        rospy.spin()

    def odom_callback(self, data: Odometry):
        out = data
        out.header.frame_id = 'world'
        out.pose.pose.position.x = data.pose.pose.position.x + self.init_x
        out.pose.pose.position.y = data.pose.pose.position.y + self.init_y
        out.pose.pose.position.z = data.pose.pose.position.z + self.init_z
        self.pub.publish(out)


if __name__ == '__main__':
    try:
        odom_server = OdomServer()
    except rospy.ROSInterruptException:
        pass
