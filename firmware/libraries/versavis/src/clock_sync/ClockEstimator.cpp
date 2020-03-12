#include "ClockEstimator.h"
#include <math.h>
void predictX(float dt, float u1, float x1, float x2, float x3, float x4, float *out_2342040993679500210) {
   out_2342040993679500210[0] = dt*x2 + dt*x4*(0.00322580645F*u1 - x3) + x1;
   out_2342040993679500210[1] = x2;
   out_2342040993679500210[2] = x3;
   out_2342040993679500210[3] = x4;
}
void predictP(float dt, float p11, float p12, float p13, float p14, float p21, float p22, float p23, float p24, float p31, float p32, float p33, float p34, float p41, float p42, float p43, float p44, float u1, float var_x1, float var_x2, float var_x3, float var_x4, float x3, float x4, float *out_2872673569952136372) {
   out_2872673569952136372[0] = dt*p21 - dt*p31*x4 + dt*p41*(0.00322580645F*u1 - x3) - dt*x4*(dt*p23 - dt*p33*x4 + dt*p43*(0.00322580645F*u1 - x3) + p13) + dt*(0.00322580645F*u1 - x3)*(dt*p24 - dt*p34*x4 + dt*p44*(0.00322580645F*u1 - x3) + p14) + dt*(dt*p22 - dt*p32*x4 + dt*p42*(0.00322580645F*u1 - x3) + p12) + p11 + var_x1;
   out_2872673569952136372[1] = dt*p22 - dt*p32*x4 + dt*p42*(0.00322580645F*u1 - x3) + p12;
   out_2872673569952136372[2] = dt*p23 - dt*p33*x4 + dt*p43*(0.00322580645F*u1 - x3) + p13;
   out_2872673569952136372[3] = dt*p24 - dt*p34*x4 + dt*p44*(0.00322580645F*u1 - x3) + p14;
   out_2872673569952136372[4] = dt*p22 - dt*p23*x4 + dt*p24*(0.00322580645F*u1 - x3) + p21;
   out_2872673569952136372[5] = p22 + var_x2;
   out_2872673569952136372[6] = p23;
   out_2872673569952136372[7] = p24;
   out_2872673569952136372[8] = dt*p32 - dt*p33*x4 + dt*p34*(0.00322580645F*u1 - x3) + p31;
   out_2872673569952136372[9] = p32;
   out_2872673569952136372[10] = p33 + var_x3;
   out_2872673569952136372[11] = p34;
   out_2872673569952136372[12] = dt*p42 - dt*p43*x4 + dt*p44*(0.00322580645F*u1 - x3) + p41;
   out_2872673569952136372[13] = p42;
   out_2872673569952136372[14] = p43;
   out_2872673569952136372[15] = p44 + var_x4;
}
void computeResidual(float x1, float z1, float *out_7219536492507892286) {
   out_7219536492507892286[0] = -x1 + z1;
}
void computeSInverse(float p11, float var_z1, float *out_3216141876827372337) {
   out_3216141876827372337[0] = 1.0F/(p11 + var_z1);
}
void computeK(float p11, float p21, float p31, float p41, float s_inv11, float *out_5112816909577445259) {
   out_5112816909577445259[0] = p11*s_inv11;
   out_5112816909577445259[1] = p21*s_inv11;
   out_5112816909577445259[2] = p31*s_inv11;
   out_5112816909577445259[3] = p41*s_inv11;
}
void estimateX(float k1, float k2, float k3, float k4, float x1, float x2, float x3, float x4, float z1, float *out_6534339328614102993) {
   out_6534339328614102993[0] = k1*(-x1 + z1) + x1;
   out_6534339328614102993[1] = k2*(-x1 + z1) + x2;
   out_6534339328614102993[2] = k3*(-x1 + z1) + x3;
   out_6534339328614102993[3] = k4*(-x1 + z1) + x4;
}
void estimateP(float k1, float k2, float k3, float k4, float p11, float p12, float p13, float p14, float p21, float p22, float p23, float p24, float p31, float p32, float p33, float p34, float p41, float p42, float p43, float p44, float *out_1119496156146403128) {
   out_1119496156146403128[0] = p11*(1 - k1);
   out_1119496156146403128[1] = p12*(1 - k1);
   out_1119496156146403128[2] = p13*(1 - k1);
   out_1119496156146403128[3] = p14*(1 - k1);
   out_1119496156146403128[4] = -k2*p11 + p21;
   out_1119496156146403128[5] = -k2*p12 + p22;
   out_1119496156146403128[6] = -k2*p13 + p23;
   out_1119496156146403128[7] = -k2*p14 + p24;
   out_1119496156146403128[8] = -k3*p11 + p31;
   out_1119496156146403128[9] = -k3*p12 + p32;
   out_1119496156146403128[10] = -k3*p13 + p33;
   out_1119496156146403128[11] = -k3*p14 + p34;
   out_1119496156146403128[12] = -k4*p11 + p41;
   out_1119496156146403128[13] = -k4*p12 + p42;
   out_1119496156146403128[14] = -k4*p13 + p43;
   out_1119496156146403128[15] = -k4*p14 + p44;
}
