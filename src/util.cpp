#include <vfh_rover/util.h>

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

void add(float* x, float y, float* ret_val, int count) {
  for (int i = 0; i < count; i++) {
    ret_val[i] = x[i] + y;
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
