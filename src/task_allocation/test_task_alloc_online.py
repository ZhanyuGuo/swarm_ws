"""
Performs task allocation given number of drones online
"""
import numpy as np

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


MAX_POS = 15
MIN_POS = -15

NUM_ROBOTS = 10
ROBOT_INIT_POSES = [
    [0.0, -1.0, 0.0],
    [0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [1.0, -1.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 1.0, 0.0],
    [2.0, -1.0, 0.0],
    [2.0, 0.0, 0.0],
    [2.0, 1.0, 0.0],
    [2.0, 2.0, 0.0],
]


TASK_LIST = [
    # (8, 8, 1),
    (8.5, 8.5, 1.5),
    (8.5, 7.5, 2.0),
    (7.5, 8.5, 2.5),
    (7.5, 7.5, 3.0),
    # (9, -7, 1),
    (9.5, -6.5, 1.5),
    (9.5, -7.5, 2.0),
    (8.5, -6.5, 2.5),
    (8.5, -7.5, 3.0),
    # (-10, 9, 1),
    (-9.5, 9.5, 1.5),
    (-9.5, 8.5, 2.0),
    (-10.5, 9.5, 2.5),
    (-10.5, 8.5, 3.0),
    # (-9, -8, 1),
    (-8.5, -7.5, 1.5),
    (-8.5, -8.5, 2.0),
    (-9.5, -7.5, 2.5),
    (-9.5, -8.5, 3.0),
]
NUM_TASK = len(TASK_LIST)

# using 1 and 0 to represent true or false
task_status = np.zeros(len(TASK_LIST))  # recording finished tasks
robot_curr_task_id = np.empty(NUM_ROBOTS)  # records which task is currently working on
rem_task_list = np.empty(len(TASK_LIST))
first_task = np.ones(NUM_ROBOTS)
task_done = np.zeros(NUM_ROBOTS)
mission_done = np.zeros(NUM_ROBOTS)
task_count = 0

robot_poses = np.empty((NUM_ROBOTS, 3))

# publisher for tasks
curr_task_pub0 = rospy.Publisher("/uav0_curr_task", PoseStamped, queue_size=1)
curr_task_pub1 = rospy.Publisher("/uav1_curr_task", PoseStamped, queue_size=1)
curr_task_pub2 = rospy.Publisher("/uav2_curr_task", PoseStamped, queue_size=1)
curr_task_pub3 = rospy.Publisher("/uav3_curr_task", PoseStamped, queue_size=1)
curr_task_pub4 = rospy.Publisher("/uav4_curr_task", PoseStamped, queue_size=1)
curr_task_pub5 = rospy.Publisher("/uav5_curr_task", PoseStamped, queue_size=1)
curr_task_pub6 = rospy.Publisher("/uav6_curr_task", PoseStamped, queue_size=1)
curr_task_pub7 = rospy.Publisher("/uav7_curr_task", PoseStamped, queue_size=1)
curr_task_pub8 = rospy.Publisher("/uav8_curr_task", PoseStamped, queue_size=1)
curr_task_pub9 = rospy.Publisher("/uav9_curr_task", PoseStamped, queue_size=1)
curr_task_pub = [
    curr_task_pub0,
    curr_task_pub1,
    curr_task_pub2,
    curr_task_pub3,
    curr_task_pub4,
    curr_task_pub5,
    curr_task_pub6,
    curr_task_pub7,
    curr_task_pub8,
    curr_task_pub9,
]

# publisher for mission completion
mission_complete_pub = rospy.Publisher("/mission_bool", Bool, queue_size=1)


def assign_tasks(_n_robot, r_pos, _tasks):
    task_id = np.empty(len(_tasks))  # which robot gets the task
    for t in range(len(_tasks)):
        cost_list = np.zeros(_n_robot)
        for n in range(_n_robot):
            tx, ty, tz = _tasks[t]
            rx, ry, rz = r_pos[n]
            # rospy.loginfo(rx)
            cost = (tx - rx) * (tx - rx) + (ty - ry) * (ty - ry)
            cost_list[n] = cost
        task_id[t] = np.argmin(cost_list)
    robot_tasks = []
    for n in range(_n_robot):
        robot_tasks.append([])
    for t in range(len(task_id)):
        r_id = task_id[t]
        robot_tasks[int(r_id)].append(_tasks[t])
    return task_id, robot_tasks


def online_task_assign(r_pos, _tasks, _task_status):
    rx, ry, rz = r_pos
    cost_list = np.empty(len(_tasks))
    _count = 0
    for i, t in enumerate(_tasks):
        if _task_status[i] == 0:  # task is available
            tx, ty, tz = t
            cost_list[i] = (tx - rx) * (tx - rx) + (ty - ry) * (ty - ry)
        else:  # 1 or 2
            cost_list[i] = 2 * (MAX_POS - MIN_POS) * (MAX_POS - MIN_POS)  # max possible cost to prevent assignment
            _count += 1
    if _count == NUM_TASK:
        _no_task = True
    elif _count >= NUM_TASK:
        _no_task = True
        rospy.loginfo("WARNING: COUNT IS BIGGER THAN TASK COUNT!")
    else:
        _no_task = False
    task_id = np.argmin(cost_list)
    next_task = _tasks[task_id]
    return int(task_id), next_task, _no_task


def status_callback0(data):
    # rospy.loginfo('robot 0 idling: '+str(data))
    if data.data:
        rospy.loginfo("received robot 0 idling")
        global task_done
        task_done[0] = 1


def status_callback1(data):
    # rospy.loginfo('robot 1 idling: '+str(data))
    if data.data:
        rospy.loginfo("received robot 1 idling")
        global task_done
        task_done[1] = 1


def status_callback2(data):
    # rospy.loginfo('robot 2 idling: '+str(data))
    if data.data:
        rospy.loginfo("received robot 2 idling")
        global task_done
        task_done[2] = 1


def status_callback3(data):
    # rospy.loginfo('robot 2 idling: '+str(data))
    if data.data:
        rospy.loginfo("received robot 3 idling")
        global task_done
        task_done[3] = 1


def status_callback4(data):
    # rospy.loginfo('robot 2 idling: '+str(data))
    if data.data:
        rospy.loginfo("received robot 4 idling")
        global task_done
        task_done[4] = 1


def status_callback5(data):
    # rospy.loginfo('robot 2 idling: '+str(data))
    if data.data:
        rospy.loginfo("received robot 5 idling")
        global task_done
        task_done[5] = 1


def status_callback6(data):
    # rospy.loginfo('robot 2 idling: '+str(data))
    if data.data:
        rospy.loginfo("received robot 6 idling")
        global task_done
        task_done[6] = 1


def status_callback7(data):
    # rospy.loginfo('robot 2 idling: '+str(data))
    if data.data:
        rospy.loginfo("received robot 7 idling")
        global task_done
        task_done[7] = 1


def status_callback8(data):
    # rospy.loginfo('robot 2 idling: '+str(data))
    if data.data:
        rospy.loginfo("received robot 8 idling")
        global task_done
        task_done[8] = 1


def status_callback9(data):
    # rospy.loginfo('robot 2 idling: '+str(data))
    if data.data:
        rospy.loginfo("received robot 9 idling")
        global task_done
        task_done[9] = 1


def robot0_pos_callback(msg):
    robot_poses[0][0] = msg.pose.position.x + ROBOT_INIT_POSES[0][0]
    robot_poses[0][1] = msg.pose.position.y + ROBOT_INIT_POSES[0][1]
    robot_poses[0][2] = msg.pose.position.z + ROBOT_INIT_POSES[0][2]


def robot1_pos_callback(msg):
    robot_poses[1][0] = msg.pose.position.x + ROBOT_INIT_POSES[1][0]
    robot_poses[1][1] = msg.pose.position.y + ROBOT_INIT_POSES[1][1]
    robot_poses[1][2] = msg.pose.position.z + ROBOT_INIT_POSES[1][2]


def robot2_pos_callback(msg):
    robot_poses[2][0] = msg.pose.position.x + ROBOT_INIT_POSES[2][0]
    robot_poses[2][1] = msg.pose.position.y + ROBOT_INIT_POSES[2][1]
    robot_poses[2][2] = msg.pose.position.z + ROBOT_INIT_POSES[2][2]


def robot3_pos_callback(msg):
    robot_poses[3][0] = msg.pose.position.x + ROBOT_INIT_POSES[3][0]
    robot_poses[3][1] = msg.pose.position.y + ROBOT_INIT_POSES[3][1]
    robot_poses[3][2] = msg.pose.position.z + ROBOT_INIT_POSES[3][2]


def robot4_pos_callback(msg):
    robot_poses[4][0] = msg.pose.position.x + ROBOT_INIT_POSES[4][0]
    robot_poses[4][1] = msg.pose.position.y + ROBOT_INIT_POSES[4][1]
    robot_poses[4][2] = msg.pose.position.z + ROBOT_INIT_POSES[4][2]


def robot5_pos_callback(msg):
    robot_poses[5][0] = msg.pose.position.x + ROBOT_INIT_POSES[5][0]
    robot_poses[5][1] = msg.pose.position.y + ROBOT_INIT_POSES[5][1]
    robot_poses[5][2] = msg.pose.position.z + ROBOT_INIT_POSES[5][2]


def robot6_pos_callback(msg):
    robot_poses[6][0] = msg.pose.position.x + ROBOT_INIT_POSES[6][0]
    robot_poses[6][1] = msg.pose.position.y + ROBOT_INIT_POSES[6][1]
    robot_poses[6][2] = msg.pose.position.z + ROBOT_INIT_POSES[6][2]


def robot7_pos_callback(msg):
    robot_poses[7][0] = msg.pose.position.x + ROBOT_INIT_POSES[7][0]
    robot_poses[7][1] = msg.pose.position.y + ROBOT_INIT_POSES[7][1]
    robot_poses[7][2] = msg.pose.position.z + ROBOT_INIT_POSES[7][2]


def robot8_pos_callback(msg):
    robot_poses[8][0] = msg.pose.position.x + ROBOT_INIT_POSES[8][0]
    robot_poses[8][1] = msg.pose.position.y + ROBOT_INIT_POSES[8][1]
    robot_poses[8][2] = msg.pose.position.z + ROBOT_INIT_POSES[8][2]


def robot9_pos_callback(msg):
    robot_poses[9][0] = msg.pose.position.x + ROBOT_INIT_POSES[9][0]
    robot_poses[9][1] = msg.pose.position.y + ROBOT_INIT_POSES[9][1]
    robot_poses[9][2] = msg.pose.position.z + ROBOT_INIT_POSES[9][2]


def publish_task_position(curr_task_pub0, pos):
    waypoint = PoseStamped()
    waypoint.header.stamp = rospy.Time.now()
    waypoint.pose.position.x = pos[0]
    waypoint.pose.position.y = pos[1]
    waypoint.pose.position.z = pos[2]
    curr_task_pub0.publish(waypoint)
    rospy.loginfo("current task position {}".format(pos))


if __name__ == "__main__":
    try:
        rospy.init_node("master_online_task_alloc_node", anonymous=True)

        # TODO: wait for service from robots to confirm they are ready for tasks

        # subscriber for task completion
        robot_status_sub0 = rospy.Subscriber("/uav0_task_bool", Bool, status_callback0)
        robot_status_sub1 = rospy.Subscriber("/uav1_task_bool", Bool, status_callback1)
        robot_status_sub2 = rospy.Subscriber("/uav2_task_bool", Bool, status_callback2)
        robot_status_sub3 = rospy.Subscriber("/uav3_task_bool", Bool, status_callback3)
        robot_status_sub3 = rospy.Subscriber("/uav4_task_bool", Bool, status_callback4)
        robot_status_sub3 = rospy.Subscriber("/uav5_task_bool", Bool, status_callback5)
        robot_status_sub3 = rospy.Subscriber("/uav6_task_bool", Bool, status_callback6)
        robot_status_sub3 = rospy.Subscriber("/uav7_task_bool", Bool, status_callback7)
        robot_status_sub3 = rospy.Subscriber("/uav8_task_bool", Bool, status_callback8)
        robot_status_sub3 = rospy.Subscriber("/uav9_task_bool", Bool, status_callback9)

        # subscriber for robots position
        robot_pos_sub0 = rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, robot0_pos_callback)
        robot_pos_sub1 = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, robot1_pos_callback)
        robot_pos_sub2 = rospy.Subscriber("/uav2/mavros/local_position/pose", PoseStamped, robot2_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("/uav3/mavros/local_position/pose", PoseStamped, robot3_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("/uav4/mavros/local_position/pose", PoseStamped, robot4_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("/uav5/mavros/local_position/pose", PoseStamped, robot5_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("/uav6/mavros/local_position/pose", PoseStamped, robot6_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("/uav7/mavros/local_position/pose", PoseStamped, robot7_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("/uav8/mavros/local_position/pose", PoseStamped, robot8_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("/uav9/mavros/local_position/pose", PoseStamped, robot9_pos_callback)

        rate = rospy.Rate(3)
        rospy.sleep(3)
        rospy.loginfo("planning now, robot pose:" + str(robot_poses))
        rospy.sleep(1)

        # initially assign tasks to each robot
        for n in range(NUM_ROBOTS - 1, -1, -1):
            curr_task_id, curr_task, no_task = online_task_assign(robot_poses[n], TASK_LIST, task_status)
            robot_curr_task_id[n] = int(curr_task_id)
            task_status[curr_task_id] = 1
            publish_task_position(curr_task_pub[n], curr_task)
            task_done[n] = 0
            task_count += 1
        rospy.loginfo("task assignments finished! starting mission")

        while not rospy.is_shutdown():
            for i in range(NUM_ROBOTS):  # iterate through robots
                if task_done[i] == 1:  # if robot is done with current task
                    task_status[int(robot_curr_task_id[i])] = 2
                    curr_task_id, curr_task, no_task = online_task_assign(robot_poses[i], TASK_LIST, task_status)
                    if no_task:
                        rospy.loginfo("no more task! waiting for mission to finish")
                    else:
                        robot_curr_task_id[i] = curr_task_id
                        task_status[int(robot_curr_task_id[i])] = 1
                        publish_task_position(curr_task_pub[i], curr_task)
                        task_done[i] = 0
                        task_count += 1

            # termination check
            if np.sum(task_status) == NUM_TASK * 2:
                tf = 10
                while tf > 0:  # wait to shutdown node
                    mission_complete_pub.publish(True)
                    rospy.loginfo("Mission accomplished! Terminating the node in " + str(tf))
                    tf -= 1
                    rospy.sleep(1)
                rospy.loginfo("Number of tasks completed: " + str(task_count))
                break

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
