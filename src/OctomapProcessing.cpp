#include <vfh_rover/OctomapProcessing.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

void OctomapProcessing::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
  octomap::AbstractOcTree* octree = octomap_msgs::msgToMap(*msg);
  std::cout << "Got it" << std::endl;
}
