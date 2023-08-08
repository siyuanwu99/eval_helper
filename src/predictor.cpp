#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <iostream>

int            N; /* number of obstacles */
double         x, y, z;
ros::Publisher marker_pub_;

void obsCallback(const visualization_msgs::MarkerArray::ConstPtr &msg) {
  N = msg->markers.size();
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.clear();

  // ROS_INFO_STREAM("number of obstacles: " << N);
  int id = 0;
  for (auto &mk : msg->markers) {
    double vx, vy, px, py, pz;
    px = mk.pose.position.x;
    py = mk.pose.position.y;
    pz = mk.pose.position.z;
    vx = mk.points[1].x - mk.points[0].x;
    vy = mk.points[1].y - mk.points[0].y;
    // q.w() = mk.pose.orientation.w;
    // q.x() = mk.pose.orientation.x;
    // q.y() = mk.pose.orientation.y;
    // q.z() = mk.pose.orientation.z;

    visualization_msgs::Marker mker;
    mker.header.frame_id = "world";
    mker.header.stamp    = ros::Time::now();
    mker.id              = id++;

    mker.type               = visualization_msgs::Marker::ARROW;
    mker.action             = visualization_msgs::Marker::ADD;
    mker.scale.x            = x;
    mker.scale.y            = y;
    mker.scale.y            = z;
    mker.color.r            = 1.0;
    mker.color.g            = 0.0;
    mker.color.a            = 1.0;
    mker.pose.orientation.w = 1.0;
    geometry_msgs::Point p;
    p.x = px;
    p.y = py;
    p.z = pz;
    mker.points.push_back(p);
    p.x += vx;
    p.y += vy;
    mker.points.push_back(p);
    marker_array.markers.push_back(mker);
  }
  marker_pub_.publish(marker_array);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "predictor");
  ros::NodeHandle nh("~");
  nh.param("x", x, 0.05);
  nh.param("y", y, 0.1);
  nh.param("z", z, 0.1);

  ros::Subscriber obs_sub = nh.subscribe("/ground_truth_state", 1, obsCallback);
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/ground_truth_velocity", 10);

  // while (ros::ok()) {
  //   ros::Duration(0.01).sleep();
  //   ros::spinOnce();
  // }
  ros::spin();
}
