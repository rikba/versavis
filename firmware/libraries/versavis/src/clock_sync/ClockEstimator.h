#ifndef CLOCK_SYNC__CLOCKESTIMATOR__H
#define CLOCK_SYNC__CLOCKESTIMATOR__H
void predictX(float dt, float u1, float x1, float x2, float x3, float *out_7332445746316089964);
void predictP(float dt, float p11, float p12, float p13, float p21, float p22, float p23, float p31, float p32, float p33, float u1, float var_x1, float var_x2, float var_x3, float x2, float x3, float *out_3765553124318163400);
void computeResidual(float x1, float z1, float *out_7219536492507892286);
void computeSInverse(float p11, float var_z1, float *out_3216141876827372337);
void computeK(float p11, float p21, float p31, float s_inv11, float *out_7319532328103680402);
void estimateX(float k1, float k2, float k3, float residual, float x1, float x2, float x3, float *out_5441236568728860028);
void estimateP(float k1, float k2, float k3, float p11, float p12, float p13, float p21, float p22, float p23, float p31, float p32, float p33, float *out_4217707787490338185);
#endif
