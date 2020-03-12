#ifndef CLOCK_SYNC__CLOCKESTIMATOR__H
#define CLOCK_SYNC__CLOCKESTIMATOR__H
void predictX(float dt, float u1, float x1, float x2, float x3, float x4, float *out_3024619962732513813);
void predictP(float dt, float p11, float p12, float p13, float p14, float p21, float p22, float p23, float p24, float p31, float p32, float p33, float p34, float p41, float p42, float p43, float p44, float u1, float var_x1, float var_x2, float var_x3, float var_x4, float x3, float x4, float *out_9029288405349647834);
void computeResidual(float x1, float z1, float *out_6581804215570462109);
void computeSInverse(float p11, float var_z1, float *out_4710011075268763073);
void computeK(float p11, float p21, float p31, float p41, float s_inv11, float *out_7808039221083891857);
void estimateX(float k1, float k2, float k3, float k4, float x1, float x2, float x3, float x4, float z1, float *out_4393592592736541686);
void estimateP(float k1, float k2, float k3, float k4, float p11, float p12, float p13, float p14, float p21, float p22, float p23, float p24, float p31, float p32, float p33, float p34, float p41, float p42, float p43, float p44, float *out_2655578044031403162);
#endif
