#include "ClockEstimator.h"
#include <math.h>
void predictX(float dt, float u1, float x1, float x2, float x3, float *out_7332445746316089964) {
   out_7332445746316089964[0] = dt*x3*(0.00322580645F*u1 - x2) + x1;
   out_7332445746316089964[1] = x2;
   out_7332445746316089964[2] = x3;
}
void predictP(float dt, float p11, float p12, float p13, float p21, float p22, float p23, float p31, float p32, float p33, float u1, float var_x1, float var_x2, float var_x3, float x2, float x3, float *out_3765553124318163400) {
   out_3765553124318163400[0] = -dt*p21*x3 + dt*p31*(0.00322580645F*u1 - x2) - dt*x3*(-dt*p22*x3 + dt*p32*(0.00322580645F*u1 - x2) + p12) + dt*(0.00322580645F*u1 - x2)*(-dt*p23*x3 + dt*p33*(0.00322580645F*u1 - x2) + p13) + p11 + var_x1;
   out_3765553124318163400[1] = -dt*p22*x3 + dt*p32*(0.00322580645F*u1 - x2) + p12;
   out_3765553124318163400[2] = -dt*p23*x3 + dt*p33*(0.00322580645F*u1 - x2) + p13;
   out_3765553124318163400[3] = -dt*p22*x3 + dt*p23*(0.00322580645F*u1 - x2) + p21;
   out_3765553124318163400[4] = p22 + var_x2;
   out_3765553124318163400[5] = p23;
   out_3765553124318163400[6] = -dt*p32*x3 + dt*p33*(0.00322580645F*u1 - x2) + p31;
   out_3765553124318163400[7] = p32;
   out_3765553124318163400[8] = p33 + var_x3;
}
void computeResidual(float x1, float z1, float *out_7219536492507892286) {
   out_7219536492507892286[0] = -x1 + z1;
}
void computeSInverse(float p11, float var_z1, float *out_3216141876827372337) {
   out_3216141876827372337[0] = 1.0F/(p11 + var_z1);
}
void computeK(float p11, float p21, float p31, float s_inv11, float *out_7319532328103680402) {
   out_7319532328103680402[0] = p11*s_inv11;
   out_7319532328103680402[1] = p21*s_inv11;
   out_7319532328103680402[2] = p31*s_inv11;
}
void estimateX(float k1, float k2, float k3, float residual, float x1, float x2, float x3, float *out_5441236568728860028) {
   out_5441236568728860028[0] = k1*residual + x1;
   out_5441236568728860028[1] = k2*residual + x2;
   out_5441236568728860028[2] = k3*residual + x3;
}
void estimateP(float k1, float k2, float k3, float p11, float p12, float p13, float p21, float p22, float p23, float p31, float p32, float p33, float *out_4217707787490338185) {
   out_4217707787490338185[0] = p11*(1 - k1);
   out_4217707787490338185[1] = p12*(1 - k1);
   out_4217707787490338185[2] = p13*(1 - k1);
   out_4217707787490338185[3] = -k2*p11 + p21;
   out_4217707787490338185[4] = -k2*p12 + p22;
   out_4217707787490338185[5] = -k2*p13 + p23;
   out_4217707787490338185[6] = -k3*p11 + p31;
   out_4217707787490338185[7] = -k3*p12 + p32;
   out_4217707787490338185[8] = -k3*p13 + p33;
}
