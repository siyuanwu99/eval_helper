#!/usr/bin/env python3
"""
Date: 2023-01-18
Author: Siyuan Wu 
Email: siyuanwu99@gmail.com
"""

import numpy as np
import sys
import os
import re
import matplotlib.pyplot as plt

file_root = "/home/siyuan/.ros/log/latest/"
file_name = "multi_eval-1-stdout.log"

file_path = os.path.join(file_root, file_name)


def read_file(path, data):
    with open(path, "r") as f:
        for line in f.readlines():
            if re.search(r"\[uav\d\] t", line):
                # print(line)
                # if "[uav" in line:
                id = int(re.search(r"uav(\d)", line).group(1))
                t = re.search(r"t: (\d+\.?\d*),", line).group(1)
                if float(t) <= 1e-3:
                    continue

                pos_group = re.search(
                    r"pos: ([-+]?\d+\.?\d*), ([-+]?\d+\.?\d*), ([-+]?\d+\.?\d*)",
                    line,
                )
                vel_group = re.search(
                    r"vel: ([-+]?\d+\.?\d*), ([-+]?\d+\.?\d*), ([-+]?\d+\.?\d*)",
                    line,
                )
                acc_group = re.search(
                    r"acc: ([-+]?\d+\.?\d*), ([-+]?\d+\.?\d*), ([-+]?\d+\.?\d*)",
                    line,
                )
                yaw = re.search(r"yaw: ([-+]?\d+\.?\d*),", line).group(1)
                yaw_rate = re.search(r"yaw_rate: ([-+]?\d+\.?\d*)", line).group(1)
                min_obs_dist = re.search(r"min_d_obs: ([-+]?\d+\.?\d*)", line).group(1)
                min_uav_dist = re.search(r"min_d_uav: ([-+]?\d+\.?\d*)", line).group(1)

                data[id].append(
                    np.array(
                        [
                            float(t),
                            float(pos_group.group(1)),
                            float(pos_group.group(2)),
                            float(pos_group.group(3)),
                            float(vel_group.group(1)),
                            float(vel_group.group(2)),
                            float(vel_group.group(3)),
                            float(acc_group.group(1)),
                            float(acc_group.group(2)),
                            float(acc_group.group(3)),
                            float(yaw),
                            float(yaw_rate),
                            float(min_obs_dist),
                            float(min_uav_dist),
                        ]
                    )
                )


def get_sum_control_efforts(d):
    a = d[:, 7:10]
    t = d[:, 0]
    dt = np.diff(t).reshape(-1, 1) + 1e-6
    d = np.diff(a, axis=0) / dt
    c = np.square(d).sum(axis=1).dot(dt)
    return np.sum(c)


def get_avg_flight_time(d):
    v = np.square(d[:, 4:7]).sum(axis=1)
    t0 = d[0, 0]
    i_f = len(v) - (np.flip(v) != 0).argmax()
    tf = d[i_f - 1, 0]
    return tf - t0


def get_no_path_collision(d):
    dist = d[:, 12]
    danger_idx = np.where(dist < 0.0)[0]
    if len(danger_idx) == 0:
        """No collision"""
        return 0
    interval = np.where(np.diff(danger_idx) > 5)[0] + 1
    if 1 not in interval:
        interval = np.insert(interval, 0, 1)
    collision_list = np.split(danger_idx, interval)[:-1]
    collision_list = [c for c in collision_list if len(c) > 0]
    # print("danger: ", danger_idx)
    # print("gap : ", interval)
    # print("collision list: ", collision_list)

    min_dist_idx = np.array([c[np.argmin(dist[c])] for c in collision_list])
    # print("min dist: ", min_dist_idx)

    vx = d[:, 4]
    vy = d[:, 5]
    vz = d[:, 6]
    v = np.sqrt(vx**2 + vy**2 + vz**2)
    vel_zero_idx = np.where(v < 1e-3)[0]
    # print("vel 0 : ", vel_zero_idx)
    print("occurances: ", len(np.intersect1d(min_dist_idx, vel_zero_idx)))
    return len(np.intersect1d(min_dist_idx, vel_zero_idx))


def get_collision_occurance(d, idx):
    dist = d[:, idx]
    danger_idx = np.where(dist < 0.0)[0]
    if len(danger_idx) == 0:
        return 0
    n_collision = np.sum(np.diff(danger_idx) > 1) + 1
    return n_collision


def plot_x_t(data, idx):
    t = data[:, 0]
    x = data[:, idx]
    plt.plot(t, x)
    plt.show()


def plot_v_t(data):
    t = data[:, 0]
    vx = data[:, 4]
    vy = data[:, 5]
    vz = data[:, 6]
    v = np.sqrt(vx**2 + vy**2 + vz**2)
    plt.plot(t, v)
    plt.show()


def plot_a_t(data):
    t = data[:, 0]
    ax = data[:, 7]
    ay = data[:, 8]
    az = data[:, 9]
    a = np.sqrt(ax**2 + ay**2 + az**2)
    plt.plot(t, a)
    plt.show()


def plot_y_t(data):
    t = data[:, 0]
    y = data[:, 10] * 180 / np.pi
    y += [360 if i < 0 else 0 for i in y]
    plt.plot(t, y)
    plt.show()


def get_data(n, file_path):
    """
    get data from file

    :param n int: number of agents
    :param file_path [TODO:type]: [TODO:description]
    """

    data = [[] for i in range(n)]
    read_file(file_path, data)
    for (i, d) in enumerate(data):
        data_np = np.array(d)
        data[i] = data_np
    return data


def plot_reciprocal_distance(data):
    t = data[:, 0]
    d = data[:, 13]
    plt.plot(t, d)
    plt.show()


def plot_obstacle_distance(data):
    t = data[:, 0]
    d = data[:, 12]
    plt.plot(t, d)
    plt.show()


if __name__ == "__main__":
    num_agents = 4
    data = [[] for _ in range(num_agents)]

    read_file(file_path, data)
    for (i, d) in enumerate(data):
        data_np = np.array(d)
        print(data_np.shape)
        data[i] = data_np

    # plot_x_t(data[1], 1)
    # plot_a_t(data[1])
    # plot_v_t(data[1])
    n_collide = np.array([get_collision_occurance(d, 12) for d in data])
    print(n_collide)
    plot_obstacle_distance(data[0])
    # plot_reciprocal_distance(data[0])
    print(data[0][:, 13])
    print(np.min(data[0][:, 13]))
    # plot_y_t(data[1])
    # print("sum control efforts:", get_sum_control_efforts(data[1]))
    # print("avg flight time:", get_avg_flight_time(data[1]))
