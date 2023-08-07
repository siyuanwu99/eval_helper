#!/usr/bin/env python3

"""
Date: 2022-03-02
Author: Gang Chen
Email: 947089399@qq.com
---
MADER kitty
"""

import os
import time
import sys
import re
import subprocess
import argparse
import csv
import numpy as np

import calculate
from kitty_auto_test import run

parser = argparse.ArgumentParser()
parser.add_argument("--world", type=str, default="_mader_4uav_19obs", help="world name")
parser.add_argument("--iters", type=int, default="1", help="number of agents")
parser.add_argument("--num_agents", type=int, default="4", help="number of agents")
parser.add_argument(
    "--save_path", type=str, default="resultsnormal", help="folder name"
)
parser.add_argument("--waiting_time", type=float, default="20", help="waiting time")
parser.add_argument(
    "--node_name",
    type=str,
    default="/SQ0/mader_commands",
    help="key planning node name",
)
parser.add_argument("--no-run", action="store_true")

parser.add_argument(
    "--cmd_source",
    type=str,
    default="source /home/siyuan/workspace/dyn_env_mader/devel/setup.bash",
)
parser.add_argument(
    "--cmd_recorder",
    type=str,
    default="roslaunch eval_helper eval.launch",
)
parser.add_argument(
    "--cmd_launch",
    type=str,
    default="roslaunch mader test_4_drones.launch rviz:=false",
)
parser.add_argument(
    "--cmd_trigger", type=str, default="roslaunch mader multi_goal.launch"
)

if __name__ == "__main__":
    args = parser.parse_args()
    launch_cmds = [
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=true obs_num:=10 map_mode:=0",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=false obs_num:=20 map_mode:=0",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=false obs_num:=30 map_mode:=0",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=false obs_num:=40 map_mode:=0",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=false obs_num:=50 map_mode:=0",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=true map_mode:=1",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=true map_mode:=2",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=true obs_num:=10 map_mode:=1",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=true obs_num:=10 map_mode:=2",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=true obs_num:=30 map_mode:=1",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=true obs_num:=30 map_mode:=2",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=true obs_num:=40 map_mode:=1",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=true obs_num:=40 map_mode:=2",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=true obs_num:=50 map_mode:=1",
        # "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=true obs_num:=50 map_mode:=2",
        # "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=true obs_num:=10",
        # "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=false obs_num:=20",
        # "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=false obs_num:=30",
        # "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=false obs_num:=40",
        # "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=false obs_num:=50",
        # "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=true obs_num:=5",
        # "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=true obs_num:=15",
        # "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=true obs_num:=25",
        # "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=true obs_num:=35",
        # "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=true obs_num:=45",
        "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=false obs_num:=50",
        "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=false obs_num:=40",
        "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=false obs_num:=30",
        "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=false obs_num:=20",
        "roslaunch mader sim_fkpcp_4_case_0.launch rviz:=false obs_num:=10",
        "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=false obs_num:=50",
        "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=false obs_num:=40",
        "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=false obs_num:=30",
        "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=false obs_num:=20",
        "roslaunch mader sim_fkpcp_4_case_3.launch rviz:=false obs_num:=10",
        "roslaunch mader sim_fkpcp_4_case_4.launch rviz:=false obs_num:=50",
        "roslaunch mader sim_fkpcp_4_case_4.launch rviz:=false obs_num:=40",
        "roslaunch mader sim_fkpcp_4_case_4.launch rviz:=false obs_num:=30",
        "roslaunch mader sim_fkpcp_4_case_4.launch rviz:=false obs_num:=20",
        "roslaunch mader sim_fkpcp_4_case_4.launch rviz:=false obs_num:=10",
        # "roslaunch mader sim_fkpcp_4_case_4.launch rviz:=false obs_num:=50",
    ]
    trigger_cmds = [
        # "rosrun mader sim_goal_publisher.py case_0",
        # "roslaunch mader multi_goal.launch action:=case_0",
        # "roslaunch mader multi_goal.launch action:=case_0",
        # "roslaunch mader multi_goal.launch action:=case_0",
        # "roslaunch mader multi_goal.launch action:=case_0",
        # "roslaunch mader multi_goal.launch action:=case_3",
        "roslaunch mader multi_goal.launch action:=case_0",
        "roslaunch mader multi_goal.launch action:=case_0",
        "roslaunch mader multi_goal.launch action:=case_0",
        "roslaunch mader multi_goal.launch action:=case_0",
        "roslaunch mader multi_goal.launch action:=case_0",
        "roslaunch mader multi_goal.launch action:=case_3",
        "roslaunch mader multi_goal.launch action:=case_3",
        "roslaunch mader multi_goal.launch action:=case_3",
        "roslaunch mader multi_goal.launch action:=case_3",
        "roslaunch mader multi_goal.launch action:=case_3",
        "roslaunch mader multi_goal.launch action:=case_4",
        "roslaunch mader multi_goal.launch action:=case_4",
        "roslaunch mader multi_goal.launch action:=case_4",
        "roslaunch mader multi_goal.launch action:=case_4",
        "roslaunch mader multi_goal.launch action:=case_4",
    ]

    world_names = [
        # "mader_4uav_case0_10obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case0_20obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case0_30obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case0_40obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case0_50obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case1_20obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case2_20obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case1_10obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case2_10obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case1_30obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case2_30obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case1_40obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case2_40obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case1_50obs_v2_a6_cls50.15_ir1.5",
        # "mader_4uav_case2_50obs_v2_a6_cls0.15_ir1.5",
        # "mader_4uav_case3_10obs_v2_a6_cls0.15",
        # "mader_4uav_case3_20obs_v2_a6_cls0.15",
        # "mader_4uav_case3_30obs_v2_a6_cls0.15",
        # "mader_4uav_case3_40obs_v2_a6_cls0.15",
        # "mader_4uav_case3_50obs_v2_a6_cls0.15",
        "mader_4uav_case0_50obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case0_40obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case0_30obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case0_20obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case0_10obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case3_50obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case3_40obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case3_30obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case3_20obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case3_10obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case4_50obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case4_40obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case4_30obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case4_20obs_v2_a6_cls0.15_ir1.5",
        "mader_4uav_case4_10obs_v2_a6_cls0.15_ir1.5",
    ]
    for i in range(args.iters):
        print("=====    Iteration: {}    =====".format(i))
        for (cmd, trigger, world_name) in zip(launch_cmds, trigger_cmds, world_names):
            print("=====    World: {}    =====".format(world_name))
            args.cmd_launch = cmd
            args.world = world_name
            args.cmd_trigger = trigger
            run(args)
            time.sleep(1)

    os.system(
        "notify-send -u critical -t 2000 --hint int:transient:1 AutoScript finished"
    )
