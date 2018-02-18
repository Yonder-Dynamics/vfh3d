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

OctomapProcessing::OctomapProcessing(float alpha, Vehicle v,
                                     float maxRange, ros::NodeHandle n) {
  this->hu = new HistogramUpdate(alpha);
  this->vehicle = v;
  this->maxRange = maxRange;
  pub = n.advertise<sensor_msgs::PointCloud2>("histogram", 2);
}

void OctomapProcessing::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
  octomap::AbstractOcTree* atree = octomap_msgs::msgToMap(*msg);
  octomap::OcTree tree = *(octomap::OcTree *)atree;
  boost::shared_ptr<octomap::OcTree> s_tree = boost::make_shared<octomap::OcTree>(tree);
  octomap::OcTree::leaf_bbx_iterator end = tree.end_leafs_bbx();
  for (float x=-5; x<5; x+=0.5) {
    vehicle.x = x;
    Histogram h = hu->build(s_tree, vehicle, maxRange, end);
    RGBPointCloud::Ptr pc = h.displayCloud(1);
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*pc, pc_msg);
    pc_msg.header.frame_id = "map";
    pub.publish(pc_msg);
    std::cout << h.displayString() << std::endl;
    usleep(1000*1000);
  }
}
