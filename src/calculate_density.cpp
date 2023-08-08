#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

#define PI 3.14159265359

double map_x, map_y, map_z, resolution, map_volume;
double num_occupied_voxels = 0;
double num_all_voxels      = 0;
double avg_density         = 0.0;
int    num_msgs;

void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
  std::cout << "Received a point cloud with " << cloud_msg->width * cloud_msg->height << " points."
            << std::endl;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_out;
  pcl::fromROSMsg(*cloud_msg, cloud);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud.makeShared());
  sor.setLeafSize(resolution, resolution, resolution);
  sor.filter(cloud_out);
  num_occupied_voxels = cloud.points.size();
  std::cout << "Number of occupied voxels: " << cloud_out.points.size() << std::endl;
  std::cout << "Density: %" << 100 * double(num_occupied_voxels) / double(num_all_voxels)
            << std::endl;
  double density = double(num_occupied_voxels) / double(num_all_voxels);
  num_msgs++;
  avg_density += (density - avg_density) / num_msgs;
}

void stateCallback(const visualization_msgs::MarkerArrayConstPtr& msg) {
  double v = 0;

  double wx, wy, wz;
  for (auto& mk : msg->markers) {
    wx = mk.scale.x;
    wy = mk.scale.y;
    wz = mk.scale.z;
    v += PI * wx * wz * wz;
  }
  std::cout << "Number of cylinders: " << msg->markers.size() << std::endl;
  std::cout << "Volume of cylinders: " << v << std::endl;
  double density = v / map_volume;
  std::cout << "Average density: " << density << std::endl;
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "calculate_density");
  ros::NodeHandle nh;
  map_x = map_y = map_z = resolution = map_volume = 0.0;
  nh.getParam("/map_generator/map/x_size", map_x);
  nh.getParam("/map_generator/map/y_size", map_y);
  nh.getParam("/map_generator/map/z_size", map_z);
  nh.getParam("/map_generator/map/resolution", resolution);
  map_z      = 4;
  map_volume = map_x * map_y * map_z;

  num_all_voxels = map_x * map_y * map_z / resolution / resolution / resolution;

  ROS_INFO("Map size: %f, %f, %f, resolution %f", map_x, map_y, map_z, resolution);

  ros::Subscriber sub = nh.subscribe("/map_generator/global_cloud", 1, mapCallback);
  ros::Subscriber state_sub =
      nh.subscribe("/map_generator/global_cylinder_state", 1, stateCallback);

  ros::spin();
}
