#include <vfh_rover/util.h>

float dot(float* x, float* y, int count) {
  float s = 0;
  for (int i = 0; i < count; i++) {
    s += x[i] * y[i];
  }
}

void scalar_add(float* vals, float val, float* ret_val, int count) {
  for (int i = 0; i < count; i++) {
    ret_val[i] = vals[i] + val;
  }
}

void add(float* x, float* y, float* ret_val, int count) {
  for (int i = 0; i < count; i++) {
    ret_val[i] = x[i] + y[i];
  }
}

void cross(float* x, float* y, float* ret_val) {
  ret_val[0] = x[1]*y[2]-x[2]*y[1];
  ret_val[1] = x[0]*y[2]-x[2]*y[0];
  ret_val[2] = x[0]*y[1]-x[1]*y[0];
}

void add(float* x, float y, float* ret_val, int count) {
  for (int i = 0; i < count; i++) {
    ret_val[i] = x[i] + y;
  }
}

void mul(float* x, float y, float* ret_val, int count) {
  for (int i = 0; i < count; i++) {
    ret_val[i] = x[i] * y;
  }
}

int modulus(int x, int modY) {
  if (x < 0)
    return (modY + x % modY) % modY;
  else
    return x % modY;
}

int clip(int x, int min, int max) {
  int c = (x > max) ? max : x;
  return (c < min) ? min : c;
}

float sum(float* x, int count) {
  float s = 0;
  for (int i = 0; i < count; i++) {
    s += x[i];
  }
  return s;
}

int maxInd(float* x, int count) {
  int maxI = 0;
  for (int i = 0; i < count; i++) {
    if (x[i] > x[maxI])
      maxI = i;
  }
  return maxI;
}

//float headingDiff(float * 
