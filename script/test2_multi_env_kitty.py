#!/usr/bin/env python3

import argparse
import csv
import os
import re
import subprocess
import sys
import time

import calculate
import numpy as np
from kitty_auto_test import run

parser = argparse.ArgumentParser()
parser.add_argument(
    "--world", type=str, default="_4uav_20obs_v2_a8_cls0.15_", help="world name"
)
parser.add_argument("--iters", type=int, default="1", help="number of agents")
parser.add_argument("--num_agents", type=int, default="8", help="number of agents")
parser.add_argument(
    "--save_path", type=str, default="resultsnoisy_", help="folder name"
)
parser.add_argument("--waiting_time", type=float, default="20", help="waiting time")
parser.add_argument(
    "--node_name",
    type=str,
    default="/drone_0_ego_planner_node",
    help="key planning node name",
)
parser.add_argument("--no-run", action="store_true")

parser.add_argument(
    "--cmd_source",
    type=str,
    default="source /home/siyuan/workspace/dyn_env_ego-swarm/devel/setup.bash",
)
parser.add_argument(
    "--cmd_recorder",
    type=str,
    default="roslaunch eval_helper eval.launch num_agents:=8",
)
parser.add_argument(
    "--cmd_launch",
    type=str,
    default="roslaunch ego_planner dyn_evaluate.launch rviz:=false",
    # default="roslaunch ego_planner dyn_evaluate.launch rviz:=true",
)
parser.add_argument("--cmd_trigger", type=str, default="rosrun eval_helper trigger")

if __name__ == "__main__":
    args = parser.parse_args()
    launch_cmds = [
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=00 time_sync_stddev:=0.00 tracking_stddev:=0.0",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=00 time_sync_stddev:=0.01 tracking_stddev:=0.0",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=00 time_sync_stddev:=0.02 tracking_stddev:=0.0",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=00 time_sync_stddev:=0.03 tracking_stddev:=0.0",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=00 time_sync_stddev:=0.04 tracking_stddev:=0.0",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=00 time_sync_stddev:=0.05 tracking_stddev:=0.0",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=00 time_sync_stddev:=0.00 tracking_stddev:=0.1",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=00 time_sync_stddev:=0.00 tracking_stddev:=0.2",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=00 time_sync_stddev:=0.00 tracking_stddev:=0.3",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=00 time_sync_stddev:=0.00 tracking_stddev:=0.4",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=00 time_sync_stddev:=0.00 tracking_stddev:=0.5",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=20 time_sync_stddev:=0.00 tracking_stddev:=0.0",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=20 time_sync_stddev:=0.01 tracking_stddev:=0.0",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=20 time_sync_stddev:=0.02 tracking_stddev:=0.0",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=20 time_sync_stddev:=0.03 tracking_stddev:=0.0",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=20 time_sync_stddev:=0.04 tracking_stddev:=0.0",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=20 time_sync_stddev:=0.05 tracking_stddev:=0.0",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=20 time_sync_stddev:=0.00 tracking_stddev:=0.1",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=20 time_sync_stddev:=0.00 tracking_stddev:=0.2",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=20 time_sync_stddev:=0.00 tracking_stddev:=0.3",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=20 time_sync_stddev:=0.00 tracking_stddev:=0.4",
        "roslaunch ego_planner sim_fkpcp_8_case_1.launch rviz:=false obs_num:=20 time_sync_stddev:=0.00 tracking_stddev:=0.5",
    ]
    world_names = [
        "ego_8uav_00obs_v2_a6_ts0.00_tr0.0_",
        "ego_8uav_00obs_v2_a6_ts0.01_tr0.0_",
        "ego_8uav_00obs_v2_a6_ts0.02_tr0.0_",
        "ego_8uav_00obs_v2_a6_ts0.03_tr0.0_",
        "ego_8uav_00obs_v2_a6_ts0.04_tr0.0_",
        "ego_8uav_00obs_v2_a6_ts0.05_tr0.0_",
        "ego_8uav_00obs_v2_a6_ts0.00_tr0.1_",
        "ego_8uav_00obs_v2_a6_ts0.00_tr0.2_",
        "ego_8uav_00obs_v2_a6_ts0.00_tr0.3_",
        "ego_8uav_00obs_v2_a6_ts0.00_tr0.4_",
        "ego_8uav_00obs_v2_a6_ts0.00_tr0.5_",
        "ego_8uav_20obs_v2_a6_ts0.00_tr0.0_",
        "ego_8uav_20obs_v2_a6_ts0.01_tr0.0_",
        "ego_8uav_20obs_v2_a6_ts0.02_tr0.0_",
        "ego_8uav_20obs_v2_a6_ts0.03_tr0.0_",
        "ego_8uav_20obs_v2_a6_ts0.04_tr0.0_",
        "ego_8uav_20obs_v2_a6_ts0.05_tr0.0_",
        "ego_8uav_20obs_v2_a6_ts0.00_tr0.1_",
        "ego_8uav_20obs_v2_a6_ts0.00_tr0.2_",
        "ego_8uav_20obs_v2_a6_ts0.00_tr0.3_",
        "ego_8uav_20obs_v2_a6_ts0.00_tr0.4_",
        "ego_8uav_20obs_v2_a6_ts0.00_tr0.5_",
    ]
    for i in range(args.iters):
        print("=====    Iteration: {}    =====".format(i))
        for cmd, world_name in zip(launch_cmds, world_names):
            print("=====    World: {}    =====".format(world_name))
            args.cmd_launch = cmd
            args.world = world_name
            run(args)
            time.sleep(1)
    os.system(
        "notify-send -u critical -t 2000 --hint int:transient:1 AutoScript finished"
    )
