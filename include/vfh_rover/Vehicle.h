#include <octomap/math/Vector3.h>
struct Vehicle {
  float x, y, z, h, w, d;
  octomath::Vector3 min() {
    return octomath::Vector3(x, y, z);
  }
  octomath::Vector3 min() {
    return octomath::Vector3(x+w, y+h, z+d);
  }
}
