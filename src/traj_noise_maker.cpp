/**
 * @file traj_noise_maker.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief generate localization noise
 * @version 1.0
 * @date 2022-02-15
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <traj_utils/MINCOTraj.h>
#include <iostream>
#include <random>

ros::Publisher noisy_traj_pub_;

/** noise */
int                              seed_;
double                           sigma_x_, sigma_y_, sigma_z_, sigma_ts_, max_tr_;
std::default_random_engine       ng_;
std::normal_distribution<double> loc_noise_x_;
std::normal_distribution<double> loc_noise_y_;
std::normal_distribution<double> loc_noise_z_;
std::normal_distribution<double> sync_noise_;

std::uniform_real_distribution<double> tracking_noise_;

/** agents **/
int                 num_agents_;
std::vector<double> sync_errors_;

void trajCallback(const traj_utils::MINCOTrajConstPtr& msg) {
  int       id      = msg->drone_id;
  ros::Time t_start = msg->start_time;

  if (id >= num_agents_) {
    ROS_ERROR_STREAM("Agent id " << id << " is out of range of " << num_agents_ << " agents.");
    return;
  }

  /* generate noise */
  double tracking_noise = tracking_noise_(ng_);
  double sync_noise     = sync_errors_[id];
  ROS_INFO_STREAM("[TrajNoiseMaker] Agent " << id << " tracking noise: " << tracking_noise
                                            << ", sync noise: " << sync_noise);
  ros::Time noisy_t_start = t_start + ros::Duration(sync_noise) + ros::Duration(tracking_noise);
  /* @NOTE: the tracking error is considered as the difference between the
   * start time of the trajectory */
  traj_utils::MINCOTraj noisy_traj;
  noisy_traj.drone_id      = msg->drone_id;
  noisy_traj.traj_id       = msg->traj_id;
  noisy_traj.order         = msg->order;
  noisy_traj.start_time    = noisy_t_start;
  noisy_traj.des_clearance = msg->des_clearance;
  noisy_traj.shape[0]      = msg->shape[0];
  noisy_traj.shape[1]      = msg->shape[1];
  noisy_traj.shape[2]      = msg->shape[2];

  noisy_traj.start_p[0] = msg->start_p[0] + loc_noise_x_(ng_);
  noisy_traj.start_p[1] = msg->start_p[1] + loc_noise_y_(ng_);
  noisy_traj.start_p[2] = msg->start_p[2] + loc_noise_z_(ng_);

  noisy_traj.start_v[0] = msg->start_v[0];
  noisy_traj.start_v[1] = msg->start_v[1];
  noisy_traj.start_v[2] = msg->start_v[2];
  noisy_traj.start_a[0] = msg->start_a[0];
  noisy_traj.start_a[1] = msg->start_a[1];
  noisy_traj.start_a[2] = msg->start_a[2];

  noisy_traj.end_p[0] = msg->end_p[0] + loc_noise_x_(ng_);
  noisy_traj.end_p[1] = msg->end_p[1] + loc_noise_y_(ng_);
  noisy_traj.end_p[2] = msg->end_p[2] + loc_noise_z_(ng_);
  noisy_traj.end_v[0] = msg->end_v[0];
  noisy_traj.end_v[1] = msg->end_v[1];
  noisy_traj.end_v[2] = msg->end_v[2];
  noisy_traj.end_a[0] = msg->end_a[0];
  noisy_traj.end_a[1] = msg->end_a[1];
  noisy_traj.end_a[2] = msg->end_a[2];

  noisy_traj.inner_x.resize(msg->inner_x.size());
  noisy_traj.inner_y.resize(msg->inner_y.size());
  noisy_traj.inner_z.resize(msg->inner_z.size());
  for (int i = 0; i < msg->inner_x.size(); ++i) {
    noisy_traj.inner_x[i] = msg->inner_x[i] + loc_noise_x_(ng_);
    noisy_traj.inner_y[i] = msg->inner_y[i] + loc_noise_y_(ng_);
    noisy_traj.inner_z[i] = msg->inner_z[i] + loc_noise_z_(ng_);
  }
  noisy_traj.duration.resize(msg->duration.size());
  for (int i = 0; i < msg->duration.size() + 1; ++i) {
    noisy_traj.duration[i] = msg->duration[i];
  }

  noisy_traj_pub_.publish(noisy_traj);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_noise_maker");
  ros::NodeHandle nh("~");

  /* load std dev from ROS parameter server */
  nh.param("noise/seed", seed_, 0);
  nh.param("noise/loc/x", sigma_x_, 0.0);
  nh.param("noise/loc/y", sigma_y_, 0.0);
  nh.param("noise/loc/z", sigma_z_, 0.0);
  nh.param("noise/time_sync", sigma_ts_, 0.0);
  nh.param("noise/tracking", max_tr_, 0.0);

  ROS_INFO("[NoiseMaker] Localization noise sigma x = %lf", sigma_x_);
  ROS_INFO("[NoiseMaker] Localization noise sigma y = %lf", sigma_y_);
  ROS_INFO("[NoiseMaker] Localization noise sigma z = %lf", sigma_z_);
  ROS_INFO("[NoiseMaker] Time sync noise sigma = %lf", sigma_ts_);
  ROS_INFO("[NoiseMaker] Tracking noise sigma = %lf", max_tr_);

  if (seed_ == 0) {
    ng_ = std::default_random_engine(std::random_device()());
  } else {
    ng_ = std::default_random_engine(seed_);
  }
  loc_noise_x_    = std::normal_distribution<double>(0.0, sigma_x_);
  loc_noise_y_    = std::normal_distribution<double>(0.0, sigma_y_);
  loc_noise_z_    = std::normal_distribution<double>(0.0, sigma_z_);
  sync_noise_     = std::normal_distribution<double>(0.0, sigma_ts_);
  tracking_noise_ = std::uniform_real_distribution<double>(0.0, max_tr_);  // uniform distribution

  nh.param("num_agents", num_agents_, 1);
  sync_errors_.resize(num_agents_);
  for (int i = 0; i < num_agents_; ++i) {
    sync_errors_[i] = sync_noise_(ng_);
  }

  ros::Subscriber traj_sub = nh.subscribe("/broadcast_traj_to_planner", 1, &trajCallback,
                                          ros::TransportHints().tcpNoDelay());
  noisy_traj_pub_ = nh.advertise<traj_utils::MINCOTraj>("/noisy_broadcast_traj_to_planner", 1);

  ros::spin();
}
