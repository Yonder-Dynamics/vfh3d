#include <vfh_rover/HistogramUpdate.h>
#include <vfh_rover/Histogram.h>
#include <math.h>
#include <vfh_rover/Vehicle.h>
#include <iostream>
#include <cmath>

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
      h.addVoxel(pos.x(), pos.y(), pos.z(), val, rad, maxRange);
      //h.addValue(pos.x(), pos.y(), pos.z(), val);
      cnt++;
    } else {
      ign++;
    }
  }
  std::cout << "Count: " << cnt << " Ignored: " << ign << std::endl;
  return h;
}

void HistogramUpdate::binarize(Histogram& primary, int range) {
  float val, tHighB, tLowB, tHigh, tLow, meanArea, ratio;
  meanArea = primary.getMeanArea();
  tHighB = primary.mean() + (range*primary.std());
  tLowB = primary.mean() - (range*primary.std());

  std::cout << "Thresholds: "<<tHighB << "---------------" << tLowB << std::endl;

  for(int j=0; j<primary.getHeight(); j++) {
    //ratio = primary.getArea(j) / meanArea;
    ratio = 1;
    std::cout << "Ratio: "<<ratio << "----------------" << std::endl;
    tLow = tHighB * ratio;
    tHigh = tLowB * ratio;
    for(int i = 0; i<primary.getWidth(); i++) {
      val = primary.getValue(i, j);
      if(val > tHigh)
        primary.setValue(i, j, 1.0);
      else if(val < tLow)
        primary.setValue(i, j, 0.0);
      else if(j != 0)
        primary.setValue(i, j, primary.getValue(i-1, j));
      else
        primary.setValue(i, j, (abs(val-tLow) < abs(val-tHigh)) ? 0 : 1);
    }
  }
}
