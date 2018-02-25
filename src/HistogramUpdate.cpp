#include <vfh_rover/HistogramUpdate.h>
#include <vfh_rover/Histogram.h>
#include <math.h>
#include <vfh_rover/Vehicle.h>
#include <iostream>

HistogramUpdate::HistogramUpdate(float alpha) {
  this->alpha = alpha;
}
bool within(float v, float l, float m) {
  return v >= l && v <= m;
}

Histogram HistogramUpdate::build(boost::shared_ptr<octomap::OcTree> tree, Vehicle v,
                                 float maxRange, octomap::OcTree::leaf_bbx_iterator end) {
  octomath::Vector3 min (v.min().x()-maxRange,
                         v.min().y()-maxRange,
                         v.min().z()-maxRange);
  octomath::Vector3 max (v.max().x()+maxRange,
                         v.max().y()+maxRange,
                         v.max().z()+maxRange);
  Histogram h(alpha, v.x, v.y, v.z);

  // init variables for calculations
  float res = tree->getResolution();
  float rad = (res)+v.radius()+v.safety_radius; // voxel radius

  int ign=0, cnt=0;
  //for (octomap::OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(min, max),
  //     end=tree->end_leafs_bbx(); it!=end; ++it) {

  for (octomap::OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(min, max, 14) ;
       it!=end; it++) {
    octomath::Vector3 pos = it.getCoordinate();
    float val = (it->getValue()>0) ? it->getValue() : 0;
    //float val = it->getValue();
    //h.addValue(pos.x(), pos.y(), pos.z(), val);
    if (!h.isIgnored(pos.x(), pos.y(), pos.z(), maxRange)) {
      //h.addVoxel(pos.x(), pos.y(), pos.z(), val, rad, maxRange);
      h.addVoxel(pos.x(), pos.y(), pos.z(), val);
      h.checkTurning(pos.x(), pos.y(), pos.z(), val, v, rad);
      //h.addValue(pos.x(), pos.y(), pos.z(), val);
      cnt++;
    } else {
      ign++;
    }
  }
  std::cout << "Count: " << cnt << " Ignored: " << ign << std::endl;
  return h;
}
