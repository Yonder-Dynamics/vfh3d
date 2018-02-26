#pragma once

void scalar_add(float* vals, float val, float* ret_val, int count);
void add(float* x, float* y, float* ret_val, int count);
void add(float* x, float y, float* ret_val, int count);
int modulus(int x, int modY);
int clip(int x, int min, int max);
float sum(float* x, int count);
