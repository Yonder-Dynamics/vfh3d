#pragma once
#include <octomap/math/Vector3.h>
#include <algorithm>
#include <Eigen/Geometry>

struct Vehicle {
  float x, y, z, h, w, d, safety_radius;
  Eigen::Quaternionf orientation;

  float getHeading() {
    orientation.normalize();
    Eigen::Matrix3f mat = orientation.toRotationMatrix();
    Eigen::Vector3f vec = mat.eulerAngles(0,1,2);
    return vec(2);
  }

  float radius() {
    float max_dim = std::max(std::max(h,w),d);
    return max_dim/2;
  }

  float turningRadiusR() {
    return 0.2;
  }

  float turningRadiusL() {
    return 0.2;
  }
  octomath::Vector3 center() {
    return octomath::Vector3(x, y, z);
  }

  octomath::Vector3 min() {
    return octomath::Vector3(x-w/2, y-h/2, z-d/2);
  }

  octomath::Vector3 max() {
    return octomath::Vector3(x+w/2, y+h/2, z+d/2);
  }
};
