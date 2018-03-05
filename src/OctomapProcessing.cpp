#include <vfh_rover/OctomapProcessing.h>
#include <vfh_rover/HistogramUpdate.h>
#include <vfh_rover/Histogram.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <unistd.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

bool HAS_PROC = false; // temp var, only process once. For debugging

OctomapProcessing::OctomapProcessing(float alpha, Vehicle v,
                                     float maxRange, ros::NodeHandle n) :
  vehicle(v), maxRange(maxRange), gotGoal(false), gotOcto(false)
{
  this->hu = new HistogramUpdate(alpha);
  histogram_pub = n.advertise<sensor_msgs::PointCloud2>("histogram", 2);
  pose_pub = n.advertise<geometry_msgs::PoseArray>("open_poses", 2);
  next_pose_pub = n.advertise<geometry_msgs::PoseStamped>("nextPose", 2);
}

void OctomapProcessing::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  goal = msg->pose;
  gotGoal = true;
  std::cout << "Got goal" << std::endl;
  if (not HAS_PROC && gotOcto) {
    process();
  }
}

void OctomapProcessing::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
  std::cout << "Got octomap" << std::endl;
  octomap::AbstractOcTree* atree = octomap_msgs::msgToMap(*msg);
  tree = boost::make_shared<octomap::OcTree>(*(octomap::OcTree *)atree);
  gotOcto = true;
  if (not HAS_PROC && gotGoal) {
    process();
  }
}

void OctomapProcessing::process() {
  HAS_PROC = true;
  geometry_msgs::Pose * prevPose = NULL;
  for (float x=-5; x<5; x+=0.5) {
    // Build
    vehicle.x = x;
    Histogram h = hu->build(tree, vehicle, maxRange);
    //hu->binarize(h, 1);
    //std::cout << h.displayString() << std::endl;
    // Disp histogram point cloud
    RGBPointCloud::Ptr pc = h.displayCloud(1);
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*pc, pc_msg);
    pc_msg.header.frame_id = "map";
    histogram_pub.publish(pc_msg);
    // Disp string

    // Disp found free positions
    geometry_msgs::PoseArray pa;
    pa.poses = h.findPaths(5, 4);
    pa.header.frame_id = "map";
    pose_pub.publish(pa);

    // Disp next position
    geometry_msgs::PoseStamped p;
    p.pose = h.optimalPath(prevPose, vehicle, goal, 1, 0, 0);
    prevPose = &p.pose;
    p.header.frame_id = "map";
    next_pose_pub.publish(p);

    usleep(1000*1000);
  }
}
