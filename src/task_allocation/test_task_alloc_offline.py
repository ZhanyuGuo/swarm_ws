"""
Performs task allocation given number of drones
"""
import random
import numpy as np

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


# def generate_tasks(num_task, min_pos, max_pos, height):
#     task_list = []
#     for i in range(num_task):
#         task = (random.uniform(min_pos, max_pos), random.uniform(min_pos, max_pos), height)
#         task_list.append(task)
#     return task_list


num_robots = 4
robot_init_pose = [[0.0, 1.0, -0.5], [0.0, -1.0, -0.5], [2.0, 1.0, -0.5], [2.0, -1.0, -0.5]]

# num_task = 12
# task_list = generate_tasks(num_task, -30, 30, 20)  # randomly generate tasks
# task_list = [(-9, -9, 20), (-9, 9, 20), (9, -9, 20), (9, 9, 20), (-4, -4, 20), (4, -4, 20), (-4, 4, 20), (4, 4, 20),
#              (0, 10, 20), (10, 0, 20), (0, -10, 20), (-10, 0, 20)]
# task_list = [(-9, -9, 20), (-13, 7, 20), (6, -14, 20), (9, 9, 20), (-3, -5, 20), (1, -2, 20), (-4, 14, 20), (4, 4, 20)]
task_list = [(10, -7, 0.5), (11, 6, 0.7), (7, 9, 0.2), (4, -13, 0.5), (-10, -8, 0.3), (-10, 8, 0.3), (-7, -0.5, 0.2)]

task_count = 0
first_task = np.ones(num_robots)
task_done = np.zeros(num_robots)
mission_done = np.zeros(num_robots)

robot_pos = np.empty((num_robots, 3))

# publisher for tasks to every uav
curr_task_pub0 = rospy.Publisher("/uav0_curr_task", PoseStamped, queue_size=1)
curr_task_pub1 = rospy.Publisher("/uav1_curr_task", PoseStamped, queue_size=1)
curr_task_pub2 = rospy.Publisher("/uav2_curr_task", PoseStamped, queue_size=1)
curr_task_pub3 = rospy.Publisher("/uav3_curr_task", PoseStamped, queue_size=1)
curr_task_pub = [curr_task_pub0, curr_task_pub1, curr_task_pub2, curr_task_pub3]

# # publisher for tasks to ego-swarm
# ego_curr_task_pub0 = rospy.Publisher("/move_base_simple/goal_0", PoseStamped, queue_size=1)
# ego_curr_task_pub1 = rospy.Publisher("/move_base_simple/goal_1", PoseStamped, queue_size=1)
# ego_curr_task_pub2 = rospy.Publisher("/move_base_simple/goal_2", PoseStamped, queue_size=1)
# ego_curr_task_pub3 = rospy.Publisher("/move_base_simple/goal_3", PoseStamped, queue_size=1)
# ego_curr_task_pub = [ego_curr_task_pub0, ego_curr_task_pub1, ego_curr_task_pub2, ego_curr_task_pub3]

# publisher for mission completion
mission_complete_pub = rospy.Publisher("/mission_bool", Bool, queue_size=1)


def assign_tasks(_n_robot, r_pos, _tasks):
    # which robot gets the task
    task_id = np.empty(len(_tasks))
    cost_list = np.zeros(_n_robot)
    for t in range(len(task_list)):
        for n in range(_n_robot):
            tx, ty, tz = task_list[t]
            rx, ry, rz = r_pos[n]
            cost = (tx - rx) * (tx - rx) + (ty - ry) * (ty - ry)
            cost_list[n] += cost
        # argmin robot assigned the task
        task_id[t] = np.argmin(cost_list)

    robot_tasks = []
    for n in range(_n_robot):
        robot_tasks.append([])
    for t in range(len(task_id)):
        r_id = task_id[t]
        robot_tasks[int(r_id)].append(_tasks[t])

    return task_id, robot_tasks


def status_callback0(data):
    if data.data:
        rospy.loginfo("received robot 0 idling")
        global task_done
        task_done[0] = 1


def status_callback1(data):
    if data.data:
        rospy.loginfo("received robot 1 idling")
        global task_done
        task_done[1] = 1


def status_callback2(data):
    if data.data:
        rospy.loginfo("received robot 2 idling")
        global task_done
        task_done[2] = 1


def status_callback3(data):
    if data.data:
        rospy.loginfo("received robot 3 idling")
        global task_done
        task_done[3] = 1


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


def publish_task_position(_curr_task_pub, pos):
    waypoint = PoseStamped()
    waypoint.header.stamp = rospy.Time.now()
    waypoint.pose.position.x = pos[0]
    waypoint.pose.position.y = pos[1]
    waypoint.pose.position.z = pos[2]
    _curr_task_pub.publish(waypoint)
    rospy.loginfo("current task position {}".format(pos))


if __name__ == "__main__":
    try:
        rospy.init_node("master_offline_task_alloc_node", anonymous=True)

        # TODO: wait for service from robots to confirm they are ready for tasks

        # subscriber for task completion
        robot_status_sub0 = rospy.Subscriber("/uav0_task_bool", Bool, status_callback0)
        robot_status_sub1 = rospy.Subscriber("/uav1_task_bool", Bool, status_callback1)
        robot_status_sub2 = rospy.Subscriber("/uav2_task_bool", Bool, status_callback2)
        robot_status_sub3 = rospy.Subscriber("/uav3_task_bool", Bool, status_callback3)

        # subscriber for robots position
        robot_pos_sub0 = rospy.Subscriber("uav0/mavros/local_position/pose", PoseStamped, robot0_pos_callback)
        robot_pos_sub1 = rospy.Subscriber("uav1/mavros/local_position/pose", PoseStamped, robot1_pos_callback)
        robot_pos_sub2 = rospy.Subscriber("uav2/mavros/local_position/pose", PoseStamped, robot2_pos_callback)
        robot_pos_sub3 = rospy.Subscriber("uav3/mavros/local_position/pose", PoseStamped, robot3_pos_callback)

        rate = rospy.Rate(3)
        rospy.sleep(3)
        rospy.loginfo("planning now, robot pose:" + str(robot_pos))
        rospy.sleep(3)
        task_id_list, robot_tasks_list = assign_tasks(num_robots, robot_pos, task_list)
        rospy.loginfo("task assignments finished! starting mission" + str(robot_tasks_list))

        while not rospy.is_shutdown():
            for i in range(num_robots):  # iterate through robots
                if task_done[i] == 1:  # if current task is done
                    if first_task[i] == 1:
                        first_task[i] = 0
                    else:
                        if robot_tasks_list[i]:
                            robot_tasks_list[i].pop(0)
                    if robot_tasks_list[i]:  # if there is next task
                        task_count += 1
                        curr_task = robot_tasks_list[i][0]
                        rospy.loginfo("Assigning the next task")
                        task_done[i] = 0

                        publish_task_position(curr_task_pub[i], robot_tasks_list[i][0])
                        # publish_task_position(ego_curr_task_pub[i], robot_tasks_list[i][0])
                    else:
                        rospy.loginfo("robot" + str(i) + " tasks completed! Waiting for other robots")
                        mission_done[i] = 1

            # termination check
            if sum(mission_done) == num_robots:
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
