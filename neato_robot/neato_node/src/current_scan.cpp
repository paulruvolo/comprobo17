#include "ros/ros.h"
#include "ros/time.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <math.h>

laser_geometry::LaserProjection* projector_ = 0;
tf::TransformListener* listener_ = 0;
ros::Publisher pub;

void pointCloudCallback (const sensor_msgs::PointCloud::ConstPtr& cloud_in)
{
  std::cout << "got a message!!!" <<std::endl;
  if (listener_ == 0 || projector_ == 0) {
    return;
  }
  tf::StampedTransform odomBaseLink;
  try {
   listener_->lookupTransform("odom", "base_laser_link", ros::Time(0), odomBaseLink);
  } catch (tf::TransformException ex) {
    // that's okay... we probably just haven't seen the odom frame yet
    return;
  }
  sensor_msgs::PointCloud base_link_cloud;
  sensor_msgs::PointCloud cloud_copy(*cloud_in);

  cloud_copy.header.stamp = odomBaseLink.stamp_;
  try {
    listener_->transformPointCloud("base_laser_link", cloud_copy, base_link_cloud);
  } catch (tf::TransformException ex) {
    // that's okay... we probably just haven't seen the odom frame yet
    return;
  }
  sensor_msgs::LaserScan scan_out;

  scan_out.header.stamp = odomBaseLink.stamp_;
  scan_out.header.frame_id = "base_laser_link";
  scan_out.angle_min = -M_PI;
  scan_out.angle_max = M_PI;
  scan_out.angle_increment = 2*M_PI/360.0;
  scan_out.time_increment = 0.0;      // assume instantaneous scan
  scan_out.scan_time = 0.0;           // assume instantaneous scan
  scan_out.range_min = 0.02;
  scan_out.range_max = 5.0;
  // initially populate the scan with empty data, then fill it in with the point cloud
  for (int i = 0; i < 361; i++) {
    scan_out.ranges.push_back(0.0);
    scan_out.intensities.push_back(0.0);
  }
  for (int i = 0; i < base_link_cloud.points.size(); i++) {
    float bearing = atan2(base_link_cloud.points[i].y, base_link_cloud.points[i].x);
    int idx = round((bearing - scan_out.angle_min)/(scan_out.angle_max - scan_out.angle_min)*361);
    if (idx >= 0 && idx <= 360) {
      scan_out.ranges[idx] = sqrt(base_link_cloud.points[i].x*base_link_cloud.points[i].x +
                                  base_link_cloud.points[i].y*base_link_cloud.points[i].y);
      scan_out.intensities[idx] = 10.0;
    }
  }
  pub.publish(scan_out);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "current_scan");

  listener_ = new tf::TransformListener();
  projector_ = new laser_geometry::LaserProjection();
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/projected_stable_scan",10,pointCloudCallback);
  pub = node.advertise<sensor_msgs::LaserScan>("/current_scan",10);

  ros::Rate rate(10.0);
  while (node.ok()){
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};