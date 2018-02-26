#pragma once

float dot(float* x, float* y, int count);
void scalar_add(float* vals, float val, float* ret_val, int count);
void add(float* x, float* y, float* ret_val, int count);
void cross(float* x, float* y, float* ret_val);
void add(float* x, float y, float* ret_val, int count);
void mul(float* x, float y, float* ret_val, int count);
int modulus(int x, int modY);
int clip(int x, int min, int max);
float sum(float* x, int count);
int maxInd(float *x , int count);
//void quatRot(float* q, float * x);
