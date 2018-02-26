#pragma once
#include <octomap/math/Vector3.h>
#include <algorithm>

struct Vehicle {
  float x, y, z, h, w, d, safety_radius, heading;

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
