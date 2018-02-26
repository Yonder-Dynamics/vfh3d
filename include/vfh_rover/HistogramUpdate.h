#pragma once
#include <vfh_rover/Histogram.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include <vfh_rover/Vehicle.h>

class HistogramUpdate {
  private:
    Histogram* histogram;
    float alpha;
  public:
    HistogramUpdate(float alpha);
    Histogram build(boost::shared_ptr<octomap::OcTree> tree, Vehicle v,
                    float maxRange);
};
