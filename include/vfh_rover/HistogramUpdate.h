#include <Histogram.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>

class HistogramUpdate {
  private:
    Histogram* histogram;
  public:
    HistogramUpdate(float alpha) {
      this->histogram = new Histogram::Histogram(alpha);
    };
    int enlargementAngle(float alpha, float xi, float yi, float zi, 
        float xo, float yo, float zo, float r, float s ,float v);
    void build(AbstractOcTree & input);
};
