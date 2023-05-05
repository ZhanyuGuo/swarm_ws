from __future__ import division

import math
import numpy as np

from threading import Thread
from pymavlink import mavutil

from mavros_test_common import MavrosTestCommon

import rospy
# from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import PoseStamped
# from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import PositionTarget

PKG = 'px4'


class MavrosOffboardPosctlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        # self.pos = PoseStamped()
        self.pos = PositionTarget()
        self.pos.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        self.radius = 1

        self.init_x = 2
        self.init_y = -1
        self.init_z = -0.5
        self.height = 1

        self.curr_task_pos = (self.init_x, self.init_y, self.height)
        self.prev_task_pos = (self.init_x, self.init_y, self.height)
        self.mission_done = False
        self.idling = False

        # publishers
        # self.pos_setpoint_pub = rospy.Publisher('/uav6/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.raw_setpoint_pub = rospy.Publisher('/uav6/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.idle_pub = rospy.Publisher('/uav6_task_bool', Bool, queue_size=1)

        # subscribers
        self.mission_done_sub = rospy.Subscriber('/mission_bool', Bool, self.mission_done_callback)
        self.task_pos_sub = rospy.Subscriber('/uav6_curr_task', PoseStamped, self.task_pos_callback)
        # self.uav6_pos_sub = rospy.Subscriber('/uav6/mavros/local_position/pose', PoseStamped, self.pos_callback)
        self.raw_setpoint_sub = rospy.Subscriber('/uav6/mavros/setpoint_raw/bridge', PositionTarget, self.raw_callback)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def raw_callback(self, data: PositionTarget):
        self.pos = data
        self.pos.position.x -= self.init_x
        self.pos.position.y -= self.init_y
        self.pos.position.z -= self.init_z

    def task_pos_callback(self, data):
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        new_task = (x, y, z)
        if self.curr_task_pos == new_task:
            pass
        else:
            rospy.loginfo("received new task!")
            self.prev_task_pos = self.curr_task_pos
            self.curr_task_pos = new_task
            self.idling = False

    def mission_done_callback(self, data):
        if data.data:
            self.mission_done = True
            rospy.loginfo("Received mission done from master!")

    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()

    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.raw_setpoint_pub.publish(self.pos)
            # self.pos_setpoint_pub.publish(self.pos)

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x + self.init_x,
                self.local_position.pose.position.y + self.init_y,
                self.local_position.pose.position.z + self.init_z
            ))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        # NOTE: global to local, minus offset
        self.pos.position.x = x - self.init_x
        self.pos.position.y = y - self.init_y
        self.pos.position.z = z - self.init_z
        self.pos.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
        # self.pos.pose.position.x = x - self.init_x
        # self.pos.pose.position.y = y - self.init_y
        # self.pos.pose.position.z = z - self.init_z

        # NOTE: local to global, add offset
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".format(
                x, y, z,
                self.local_position.pose.position.x + self.init_x,
                self.local_position.pose.position.y + self.init_y,
                self.local_position.pose.position.z + self.init_z
            ))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        # quaternion = quaternion_from_euler(0, 0, yaw)
        # self.pos.pose.orientation = Quaternion(*quaternion)
        self.pos.yaw = yaw

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in range(timeout * loop_freq):
            # if self.is_at_position(self.pos.pose.position.x,
            #                        self.pos.pose.position.y,
            #                        self.pos.pose.position.z, self.radius):
            if self.is_at_position(self.pos.position.x,
                                   self.pos.position.y,
                                   self.pos.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(i / loop_freq, timeout))
                self.idling = True
                self.idle_pub.publish(self.idling)
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x + self.init_x,
                   self.local_position.pose.position.y + self.init_y,
                   self.local_position.pose.position.z + self.init_z, timeout)))

    def check_task(self):
        x = self.curr_task_pos[0]
        y = self.curr_task_pos[1]
        z = self.curr_task_pos[2]
        # NOTE: global to local, minus
        if self.is_at_position(x - self.init_x, y - self.init_y, z - self.init_z, self.radius):
            self.idling = True
        else:
            self.idling = False  # get robot back to work

    #
    # Test method
    #
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        self.reach_position(self.init_x, self.init_y, self.height, 30)  # set takeoff height
        rospy.loginfo("height reached; waiting for mission")

        while not self.mission_done:
            if self.idling:
                rospy.loginfo('waiting for next task')
                self.check_task()
                # publish idling
                self.idle_pub.publish(self.idling)
                rospy.sleep(1)
            else:
                # do not need to reach position, task allocation send target to ego-planner and get discrete pose back
                # self.reach_position(self.curr_task_pos[0], self.curr_task_pos[1], self.curr_task_pos[2], 30)
                rospy.loginfo('doing task')
                self.check_task()
                self.idle_pub.publish(self.idling)
                rospy.sleep(1)
            if self.mission_done and self.idling:
                rospy.loginfo('mission completed! Landing now')
                break

        self.reach_position(self.init_x, self.init_y, self.height, 30)  # returning to home location
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        self.set_arm(False, 5)


if __name__ == '__main__':
    try:
        import rostest
        rospy.init_node('test_uav6_node', anonymous=True)

        rostest.rosrun(PKG, 'mavros_offboard_posctl_test', MavrosOffboardPosctlTest)
    except rospy.ROSInterruptException:
        pass
