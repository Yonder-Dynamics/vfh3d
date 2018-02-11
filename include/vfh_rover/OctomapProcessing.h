#include <octomap/octomap.h>
#include <octomap/math/Vector3.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

class OctomapProcessing {
 public:
  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

  octomap::AbstractOcTree* getMap();
  private:
    octomap::AbstractOcTree* input;
};
