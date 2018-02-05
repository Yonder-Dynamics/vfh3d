#include <vfh_rover/Histogram.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>

class HistogramUpdate {
  private:
    Histogram* histogram;
    float w, h, d;
  public:
    HistogramUpdate(float alpha, float w, float h, float d) {
      this->histogram = new Histogram(alpha);
      this->w = w;
      this->h = h;
      this->d = d;
    };
    int enlargementAngle(float alpha, float xi, float yi, float zi, 
        float xo, float yo, float zo, float r, float s ,float v);
    void build(octomap::AbstractOcTree & input, octomap::point3d center);
};
