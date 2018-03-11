#include <math.h>
#include <vfh_rover/VFHistogram.h>
#include <vfh_rover/Vehicle.h>
#include <string>
#include <assert.h>
#include <iostream>
#include <cmath>
#include <pcl/point_types.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

VFHistogram::VFHistogram(boost::shared_ptr<octomap::OcTree> tree, Vehicle v,
                         float maxRange, float alpha) :
  Histogram(alpha, v.x, v.y, v.z)
{
  data = new float[getWidth() * getHeight()]();
  octomath::Vector3 min (v.min().x()-maxRange,
      v.min().y()-maxRange,
      v.min().z()-maxRange);
  octomath::Vector3 max (v.max().x()+maxRange,
      v.max().y()+maxRange,
      v.max().z()+maxRange);

  // init variables for calculations
  float res = tree->getResolution();
  float rad = (res)+v.radius()+v.safety_radius; // voxel radius

  // Add voxels to tree
  int ign=0, cnt=0;
  for (octomap::OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(min, max, 14),
      end=tree->end_leafs_bbx(); it!=end; ++it) {
    octomath::Vector3 pos = it.getCoordinate();
    float val = (it->getValue()>0) ? it->getValue() : 0;
    if (!isIgnored(pos.x(), pos.y(), pos.z(), maxRange)) {
      addVoxel(pos.x(), pos.y(), pos.z(), val, rad, maxRange);
      //h.addVoxel(pos.x(), pos.y(), pos.z(), val);
      checkTurning(pos.x(), pos.y(), pos.z(), val, v, rad);
      cnt++;
    } else {
      ign++;
    }
  }
  std::cout << "Count: " << cnt << " Ignored: " << ign << std::endl;
}

void VFHistogram::addVoxel(float x, float y, float z, float val) {
  int az = getI(x, y);
  int el = getJ(x, y, z);
  setValue(az, el, getValue(az, el) + val);
}

void VFHistogram::addVoxel(float x, float y, float z, float val,
    float voxel_radius, float maxRange) {
  // For weight calculation
  float dist = sqrt(pow(x-ox, 2) +
      pow(y-oy, 2) +
      pow(z-oz, 2));
  float enlargement = floor(asin(voxel_radius/dist)/alpha);
  float a = 0.5;
  float b = 4*(a-1)/pow(maxRange-1, 2);
  float h = val*val*(a-b*(dist-voxel_radius));

  int bz = getI(x, y);
  int be = getJ(x, y, z);
  int voxelCellSize = 1; //(int)(enlargement/alpha); // divided by 2
  int az,el;
  addValues(h, bz-voxelCellSize, be-voxelCellSize, 2*voxelCellSize, 2*voxelCellSize);
  /*
  float gv[4*voxelCellSize*voxelCellSize];
  getValues(gv, bz-voxelCellSize, be-voxelCellSize, 2*voxelCellSize, 2*voxelCellSize);
  add(gv, h, gv, 4*voxelCellSize*voxelCellSize);
  setValues(gv, bz-voxelCellSize, be-voxelCellSize, 2*voxelCellSize, 2*voxelCellSize);
  */
}

void VFHistogram::checkTurning(float x, float y, float z, float val,
                             Vehicle v, float voxel_radius) {
  // Iterate over half possible ways the rover can move (then check left and right
  int j = getJ(x,y,z);
  int i = getI(x,y);
  float turningLeftCenterX = v.turningRadiusR()*sin(v.getHeading());
  float turningRightCenterX = -v.turningRadiusR()*sin(v.getHeading());
  float turningLeftCenterY = v.turningRadiusL()*cos(v.getHeading());
  float turningRightCenterY = -v.turningRadiusL()*cos(v.getHeading());
  float dr = sqrt(pow((turningRightCenterX - (x-ox)), 2) +
                  pow((turningRightCenterY - (x-ox)), 2));
  float dl = sqrt(pow((turningLeftCenterX - (x-ox)), 2) +
                  pow((turningLeftCenterY - (x-ox)), 2));
  float rad = v.safety_radius+v.radius()+voxel_radius;
  if (dr < v.turningRadiusR()+rad || dl < v.turningRadiusL()+rad)
    addValue(i, j, val);
}

std::vector<geometry_msgs::Pose> VFHistogram::findPaths(int width, int height) {
  float ret_vals[width*height];
  std::vector<geometry_msgs::Pose> ps;
  float az, el;
  for (int i=0; i<getWidth(); i++) {
    for (int j=0; j<getHeight(); j++) {
      getValues(ret_vals, i, j, width, height);
      /*
      for (int i2=0; i2<width; i2++) {
        for (int j2=0; j2<height; j2++) {
          std::cout << ret_vals[j2*width+i2] << ", ";
        }
        std::cout << std::endl;
      }
      */
      float s = sum(ret_vals, width*height);
      //std::cout << s << std::endl;
      if (abs(s) < 0.0001) {
        az = -(i+(float)width/2-getWidth()/4)*alpha;
        el = M_PI/2-(j+(float)height/2-1)*alpha;
        geometry_msgs::Pose p;
        // Abbreviations for the various angular functions
        double cy = cos(az * 0.5);
        double sy = sin(az * 0.5);
        double cr = cos(0 * 0.5); // roll = 0
        double sr = sin(0 * 0.5); // roll = 0
        double cp = cos(el * 0.5);
        double sp = sin(el * 0.5);

        p.orientation.w = cy * cr * cp + sy * sr * sp;
        p.orientation.x = cy * sr * cp - sy * cr * sp;
        p.orientation.y = cy * cr * sp + sy * sr * cp;
        p.orientation.z = sy * cr * cp - cy * sr * sp;

        p.position.x = ox;
        p.position.y = oy;
        p.position.z = oz;
        ps.push_back(p);
      }
    }
  }
  return ps;
}

geometry_msgs::Pose VFHistogram::optimalPath(geometry_msgs::Pose* prevPath, Vehicle v, geometry_msgs::Pose goal,
                                           float goalWeight, float headingWeight, float prevWeight) {
  std::vector<geometry_msgs::Pose> open = findPaths(int(v.safety_radius+v.w), int(v.safety_radius+v.h));
  float vals[open.size()];
  float dx = goal.position.x - v.x;
  float dy = goal.position.y - v.y;
  float dz = goal.position.z - v.z;
  Quaternionf goalQ =
    AngleAxisf(atan2(dy, dx), Vector3f::UnitZ()) *
    AngleAxisf(-atan2(dz, dx), Vector3f::UnitY());
  Quaternionf prevQ;
  if (prevPath != NULL) {
    prevQ = Quaternionf(prevPath->orientation.w, prevPath->orientation.x,
                        prevPath->orientation.y, prevPath->orientation.z);
  }
  for (int i = 0; i < open.size(); i++) {
    geometry_msgs::Pose * p = &open.at(i);
    Quaternionf pathQ (p->orientation.w, p->orientation.x,
                       p->orientation.y, p->orientation.z);
    float prevDiff = pathQ.angularDistance(prevQ);
    float goalDiff = pathQ.angularDistance(goalQ);
    float headDiff = pathQ.angularDistance(v.orientation);
    vals[i] = -prevDiff*prevWeight - goalDiff*goalWeight - headDiff*headingWeight;
  }
  geometry_msgs::Pose bestPath = open[maxInd(vals, open.size())];

  Quaternionf pathQ (bestPath.orientation.w, bestPath.orientation.x,
                     bestPath.orientation.y, bestPath.orientation.z);
  std::cout << pathQ.angularDistance(goalQ) << std::endl;
  return bestPath;
}

void VFHistogram::binarize(int range) {
  float val, tHighB, tLowB, tHigh, tLow, meanArea, ratio;
  meanArea = getMeanArea();
  tHighB = mean() + (range*std());
  tLowB = mean() - (range*std());

  std::cout << "Thresholds: "<<tHighB << "---------------" << tLowB << std::endl;

  for(int j=0; j<getHeight(); j++) {
    //    ratio = primary.getArea(j) / meanArea;
    ratio = 1;
    //std::cout << "Ratio: "<<ratio << "----------------" << std::endl;
    tLow = tLowB * ratio;
    tHigh = tHighB * ratio;
    for(int i = 0; i<getWidth(); i++) {
      val = getValue(i, j);
      if(val > tHigh)
        setValue(i, j, 1.0);
      else if(val < tLow)
        setValue(i, j, 0.0);
    else if(val == 0.0)
        setValue(i, j, 0.0);
      else
        setValue(i, j, (abs(val-tLow) < abs(val-tHigh)) ? 0 : 1);
    }
  }
}

bool VFHistogram::isIgnored(float x, float y, float z, float ws) {
  float dist = sqrt(pow((x-ox), 2) +
      pow((y-oy), 2) +
      pow((z-oz), 2));
  return dist > ws;
}
