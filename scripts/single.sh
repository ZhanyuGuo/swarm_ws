#!/bin/bash
# swarm_ws make under ego-swarm env, so just need to source swarm_ws
source ~/Workspace/swarm_ws/devel/setup.bash
roslaunch px4_swarm single_run_in_sim.launch
