"""
Performs task allocation given number of drones online
"""
import rospy
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import Pose, PoseStamped
import random
import numpy as np


# def generate_tasks(num_task, min_pos, max_pos, height):
#     task_list = []
#     for i in range(num_task):
#         task = (random.uniform(min_pos, max_pos), random.uniform(min_pos, max_pos), height)
#         task_list.append(task)
#     return task_list


# global variable
# num_task = 12
num_robots = 4
max_pose = 10
min_pos = -10

num_robots = 10
robot_init_pose = [
    [0.0, -1.0, -0.5],
    [0.0, 0.0, -0.5],
    [0.0, 1.0, -0.5],
    [1.0, -1.0, -0.5],
    [1.0, 0.0, -0.5],
    [1.0, 1.0, -0.5],
    [2.0, -1.0, -0.5],
    [2.0, 0.0, -0.5],
    [2.0, 1.0, -0.5],
    [-1.0, 0.0, -0.5],
]

# task_list = generate_tasks(num_task, min_pos, max_pose, 20)  # randomly generate tasks
# task_list = [(-9, -9, 20), (-9, 9, 20), (9, -9, 20), (9, 9, 20), (-4, -4, 20), (4, -4, 20),
#              (-4, 4, 20), (4, 4, 20), (0, 10, 20), (10, 0, 20), (0, -10, 20), (-10, 0, 20)]
task_list = [(10, -7, 0.5), (11, 6, 0.7), (7, 9, 0.2), (4, -13, 0.5), (-10, -8, 0.3), (-10, 8, 0.3), (-7, -0.5, 0.2)]
num_task = len(task_list)

# using 1 and 0 to represent true or false
task_status = np.zeros(len(task_list))  # recording finished tasks
robot_curr_task_id = np.empty(num_robots)  # records which task is currently working on
rem_task_list = np.empty(len(task_list))
first_task = np.ones(num_robots)
task_done = np.zeros(num_robots)
mission_done = np.zeros(num_robots)
task_count = 0

robot_pos = np.empty((num_robots, 3))

"publisher for tasks"
curr_task_pub0 = rospy.Publisher("/uav0_curr_task", PoseStamped, queue_size=1)  # publish waypoint pos
curr_task_pub1 = rospy.Publisher("/uav1_curr_task", PoseStamped, queue_size=1)  # publish waypoint pos
curr_task_pub2 = rospy.Publisher("/uav2_curr_task", PoseStamped, queue_size=1)  # publish waypoint pos
curr_task_pub3 = rospy.Publisher("/uav3_curr_task", PoseStamped, queue_size=1)  # publish waypoint pos
curr_task_pub4 = rospy.Publisher("/uav4_curr_task", PoseStamped, queue_size=1)  # publish waypoint pos
curr_task_pub5 = rospy.Publisher("/uav5_curr_task", PoseStamped, queue_size=1)  # publish waypoint pos
curr_task_pub6 = rospy.Publisher("/uav6_curr_task", PoseStamped, queue_size=1)  # publish waypoint pos
curr_task_pub7 = rospy.Publisher("/uav7_curr_task", PoseStamped, queue_size=1)  # publish waypoint pos
curr_task_pub8 = rospy.Publisher("/uav8_curr_task", PoseStamped, queue_size=1)  # publish waypoint pos
curr_task_pub9 = rospy.Publisher("/uav9_curr_task", PoseStamped, queue_size=1)  # publish waypoint pos
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

ego_curr_task_pub0 = rospy.Publisher("/move_base_simple/goal_0", PoseStamped, queue_size=1)  # publish waypoint pos
ego_curr_task_pub1 = rospy.Publisher("/move_base_simple/goal_1", PoseStamped, queue_size=1)  # publish waypoint pos
ego_curr_task_pub2 = rospy.Publisher("/move_base_simple/goal_2", PoseStamped, queue_size=1)  # publish waypoint pos
ego_curr_task_pub3 = rospy.Publisher("/move_base_simple/goal_3", PoseStamped, queue_size=1)  # publish waypoint pos
ego_curr_task_pub4 = rospy.Publisher("/move_base_simple/goal_4", PoseStamped, queue_size=1)  # publish waypoint pos
ego_curr_task_pub5 = rospy.Publisher("/move_base_simple/goal_5", PoseStamped, queue_size=1)  # publish waypoint pos
ego_curr_task_pub6 = rospy.Publisher("/move_base_simple/goal_6", PoseStamped, queue_size=1)  # publish waypoint pos
ego_curr_task_pub7 = rospy.Publisher("/move_base_simple/goal_7", PoseStamped, queue_size=1)  # publish waypoint pos
ego_curr_task_pub8 = rospy.Publisher("/move_base_simple/goal_8", PoseStamped, queue_size=1)  # publish waypoint pos
ego_curr_task_pub9 = rospy.Publisher("/move_base_simple/goal_9", PoseStamped, queue_size=1)  # publish waypoint pos
ego_curr_task_pub = [
    ego_curr_task_pub0,
    ego_curr_task_pub1,
    ego_curr_task_pub2,
    ego_curr_task_pub3,
    ego_curr_task_pub4,
    ego_curr_task_pub5,
    ego_curr_task_pub6,
    ego_curr_task_pub7,
    ego_curr_task_pub8,
    ego_curr_task_pub9,
]

"publisher for mission completion"
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
            cost_list[i] = 2 * (max_pose - min_pos) * (max_pose - min_pos)  # max possible cost to prevent assignment
            _count += 1
    if _count == num_task:
        _no_task = True
    elif _count >= num_task:
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
    robot_pos[0][0] = msg.pose.position.x + robot_init_pose[0][0]
    robot_pos[0][1] = msg.pose.position.y + robot_init_pose[0][1]
    robot_pos[0][2] = msg.pose.position.z + robot_init_pose[0][2]


def robot1_pos_callback(msg):
    robot_pos[1][0] = msg.pose.position.x + robot_init_pose[1][0]
    robot_pos[1][1] = msg.pose.position.y + robot_init_pose[1][1]
    robot_pos[1][2] = msg.pose.position.z + robot_init_pose[1][2]


def robot2_pos_callback(msg):
    robot_pos[2][0] = msg.pose.position.x + robot_init_pose[2][0]
    robot_pos[2][1] = msg.pose.position.y + robot_init_pose[2][1]
    robot_pos[2][2] = msg.pose.position.z + robot_init_pose[2][2]


def robot3_pos_callback(msg):
    robot_pos[3][0] = msg.pose.position.x + robot_init_pose[3][0]
    robot_pos[3][1] = msg.pose.position.y + robot_init_pose[3][1]
    robot_pos[3][2] = msg.pose.position.z + robot_init_pose[3][2]


def robot4_pos_callback(msg):
    robot_pos[4][0] = msg.pose.position.x + robot_init_pose[4][0]
    robot_pos[4][1] = msg.pose.position.y + robot_init_pose[4][1]
    robot_pos[4][2] = msg.pose.position.z + robot_init_pose[4][2]


def robot5_pos_callback(msg):
    robot_pos[5][0] = msg.pose.position.x + robot_init_pose[5][0]
    robot_pos[5][1] = msg.pose.position.y + robot_init_pose[5][1]
    robot_pos[5][2] = msg.pose.position.z + robot_init_pose[5][2]


def robot6_pos_callback(msg):
    robot_pos[6][0] = msg.pose.position.x + robot_init_pose[6][0]
    robot_pos[6][1] = msg.pose.position.y + robot_init_pose[6][1]
    robot_pos[6][2] = msg.pose.position.z + robot_init_pose[6][2]


def robot7_pos_callback(msg):
    robot_pos[7][0] = msg.pose.position.x + robot_init_pose[7][0]
    robot_pos[7][1] = msg.pose.position.y + robot_init_pose[7][1]
    robot_pos[7][2] = msg.pose.position.z + robot_init_pose[7][2]


def robot8_pos_callback(msg):
    robot_pos[8][0] = msg.pose.position.x + robot_init_pose[8][0]
    robot_pos[8][1] = msg.pose.position.y + robot_init_pose[8][1]
    robot_pos[8][2] = msg.pose.position.z + robot_init_pose[8][2]


def robot9_pos_callback(msg):
    robot_pos[9][0] = msg.pose.position.x + robot_init_pose[9][0]
    robot_pos[9][1] = msg.pose.position.y + robot_init_pose[9][1]
    robot_pos[9][2] = msg.pose.position.z + robot_init_pose[9][2]


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

        "subscriber for task completion"
        robot_status_sub0 = rospy.Subscriber("/uav0_task_bool", Bool, status_callback0)  # sub to robot's done with task
        robot_status_sub1 = rospy.Subscriber("/uav1_task_bool", Bool, status_callback1)
        robot_status_sub2 = rospy.Subscriber("/uav2_task_bool", Bool, status_callback2)
        robot_status_sub3 = rospy.Subscriber("/uav3_task_bool", Bool, status_callback3)
        robot_status_sub3 = rospy.Subscriber("/uav4_task_bool", Bool, status_callback4)
        robot_status_sub3 = rospy.Subscriber("/uav5_task_bool", Bool, status_callback5)
        robot_status_sub3 = rospy.Subscriber("/uav6_task_bool", Bool, status_callback6)
        robot_status_sub3 = rospy.Subscriber("/uav7_task_bool", Bool, status_callback7)
        robot_status_sub3 = rospy.Subscriber("/uav8_task_bool", Bool, status_callback8)
        robot_status_sub3 = rospy.Subscriber("/uav9_task_bool", Bool, status_callback9)

        "subscriber for robots position"
        robot_pos_sub0 = rospy.Subscriber("uav0/mavros/local_position/pose", PoseStamped, robot0_pos_callback)
        robot_pos_sub1 = rospy.Subscriber("uav1/mavros/local_position/pose", PoseStamped, robot1_pos_callback)
        robot_pos_sub2 = rospy.Subscriber("uav2/mavros/local_position/pose", PoseStamped, robot2_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("uav3/mavros/local_position/pose", PoseStamped, robot3_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("uav4/mavros/local_position/pose", PoseStamped, robot4_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("uav5/mavros/local_position/pose", PoseStamped, robot5_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("uav6/mavros/local_position/pose", PoseStamped, robot6_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("uav7/mavros/local_position/pose", PoseStamped, robot7_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("uav8/mavros/local_position/pose", PoseStamped, robot8_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("uav9/mavros/local_position/pose", PoseStamped, robot9_pos_callback)

        rate = rospy.Rate(3)
        rospy.sleep(3)
        rospy.loginfo("planning now, robot pose:" + str(robot_pos))
        rospy.sleep(1)

        # initially assign tasks to each robot
        for n in range(num_robots):
            curr_task_id, curr_task, no_task = online_task_assign(robot_pos[n], task_list, task_status)
            robot_curr_task_id[n] = int(curr_task_id)
            task_status[curr_task_id] = 1
            publish_task_position(curr_task_pub[n], curr_task)
            publish_task_position(ego_curr_task_pub[n], curr_task)
            task_done[n] = 0
            task_count += 1
        rospy.loginfo("task assignments finished! starting mission")

        while not rospy.is_shutdown():
            for i in range(num_robots):  # iterate through robots
                if task_done[i] == 1:  # if robot is done with current task
                    task_status[int(robot_curr_task_id[i])] = 2
                    curr_task_id, curr_task, no_task = online_task_assign(robot_pos[i], task_list, task_status)
                    if no_task:
                        rospy.loginfo("no more task! waiting for mission to finish")
                    else:
                        robot_curr_task_id[i] = curr_task_id
                        task_status[int(robot_curr_task_id[i])] = 1
                        publish_task_position(curr_task_pub[i], curr_task)
                        publish_task_position(ego_curr_task_pub[i], curr_task)
                        task_done[i] = 0
                        task_count += 1

            # termination check
            if np.sum(task_status) == num_task * 2:
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
