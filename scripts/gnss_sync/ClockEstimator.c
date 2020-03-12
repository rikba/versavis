#include "ClockEstimator.h"
#include <math.h>
void predictX(float dt, float u1, float x1, float x2, float x3, float x4, float *out_3024619962732513813) {
   out_3024619962732513813[0] = dt*x2 + dt*x4*(0.00322580645F*u1 - x3) + x1;
   out_3024619962732513813[1] = x2 + x4*(0.00322580645F*u1 - x3);
   out_3024619962732513813[2] = x3;
   out_3024619962732513813[3] = x4;
}
void predictP(float dt, float p11, float p12, float p13, float p14, float p21, float p22, float p23, float p24, float p31, float p32, float p33, float p34, float p41, float p42, float p43, float p44, float u1, float var_x1, float var_x2, float var_x3, float var_x4, float x3, float x4, float *out_9029288405349647834) {
   out_9029288405349647834[0] = dt*p21 - dt*p31*x4 + dt*p41*(0.00322580645F*u1 - x3) - dt*x4*(dt*p23 - dt*p33*x4 + dt*p43*(0.00322580645F*u1 - x3) + p13) + dt*(0.00322580645F*u1 - x3)*(dt*p24 - dt*p34*x4 + dt*p44*(0.00322580645F*u1 - x3) + p14) + dt*(dt*p22 - dt*p32*x4 + dt*p42*(0.00322580645F*u1 - x3) + p12) + p11 + var_x1;
   out_9029288405349647834[1] = dt*p22 - dt*p32*x4 + dt*p42*(0.00322580645F*u1 - x3) + p12 - x4*(dt*p23 - dt*p33*x4 + dt*p43*(0.00322580645F*u1 - x3) + p13) + (0.00322580645F*u1 - x3)*(dt*p24 - dt*p34*x4 + dt*p44*(0.00322580645F*u1 - x3) + p14);
   out_9029288405349647834[2] = dt*p23 - dt*p33*x4 + dt*p43*(0.00322580645F*u1 - x3) + p13;
   out_9029288405349647834[3] = dt*p24 - dt*p34*x4 + dt*p44*(0.00322580645F*u1 - x3) + p14;
   out_9029288405349647834[4] = -dt*x4*(p23 - p33*x4 + p43*(0.00322580645F*u1 - x3)) + dt*(0.00322580645F*u1 - x3)*(p24 - p34*x4 + p44*(0.00322580645F*u1 - x3)) + dt*(p22 - p32*x4 + p42*(0.00322580645F*u1 - x3)) + p21 - p31*x4 + p41*(0.00322580645F*u1 - x3);
   out_9029288405349647834[5] = p22 - p32*x4 + p42*(0.00322580645F*u1 - x3) + var_x2 - x4*(p23 - p33*x4 + p43*(0.00322580645F*u1 - x3)) + (0.00322580645F*u1 - x3)*(p24 - p34*x4 + p44*(0.00322580645F*u1 - x3));
   out_9029288405349647834[6] = p23 - p33*x4 + p43*(0.00322580645F*u1 - x3);
   out_9029288405349647834[7] = p24 - p34*x4 + p44*(0.00322580645F*u1 - x3);
   out_9029288405349647834[8] = dt*p32 - dt*p33*x4 + dt*p34*(0.00322580645F*u1 - x3) + p31;
   out_9029288405349647834[9] = p32 - p33*x4 + p34*(0.00322580645F*u1 - x3);
   out_9029288405349647834[10] = p33 + var_x3;
   out_9029288405349647834[11] = p34;
   out_9029288405349647834[12] = dt*p42 - dt*p43*x4 + dt*p44*(0.00322580645F*u1 - x3) + p41;
   out_9029288405349647834[13] = p42 - p43*x4 + p44*(0.00322580645F*u1 - x3);
   out_9029288405349647834[14] = p43;
   out_9029288405349647834[15] = p44 + var_x4;
}
void computeResidual(float x1, float z1, float *out_6581804215570462109) {
   out_6581804215570462109[0] = -x1 + z1;
}
void computeSInverse(float p11, float var_z1, float *out_4710011075268763073) {
   out_4710011075268763073[0] = 1.0F/(p11 + var_z1);
}
void computeK(float p11, float p21, float p31, float p41, float s_inv11, float *out_7808039221083891857) {
   out_7808039221083891857[0] = p11*s_inv11;
   out_7808039221083891857[1] = p21*s_inv11;
   out_7808039221083891857[2] = p31*s_inv11;
   out_7808039221083891857[3] = p41*s_inv11;
}
void estimateX(float k1, float k2, float k3, float k4, float x1, float x2, float x3, float x4, float z1, float *out_4393592592736541686) {
   out_4393592592736541686[0] = k1*(-x1 + z1) + x1;
   out_4393592592736541686[1] = k2*(-x1 + z1) + x2;
   out_4393592592736541686[2] = k3*(-x1 + z1) + x3;
   out_4393592592736541686[3] = k4*(-x1 + z1) + x4;
}
void estimateP(float k1, float k2, float k3, float k4, float p11, float p12, float p13, float p14, float p21, float p22, float p23, float p24, float p31, float p32, float p33, float p34, float p41, float p42, float p43, float p44, float *out_2655578044031403162) {
   out_2655578044031403162[0] = p11*(1 - k1);
   out_2655578044031403162[1] = p12*(1 - k1);
   out_2655578044031403162[2] = p13*(1 - k1);
   out_2655578044031403162[3] = p14*(1 - k1);
   out_2655578044031403162[4] = -k2*p11 + p21;
   out_2655578044031403162[5] = -k2*p12 + p22;
   out_2655578044031403162[6] = -k2*p13 + p23;
   out_2655578044031403162[7] = -k2*p14 + p24;
   out_2655578044031403162[8] = -k3*p11 + p31;
   out_2655578044031403162[9] = -k3*p12 + p32;
   out_2655578044031403162[10] = -k3*p13 + p33;
   out_2655578044031403162[11] = -k3*p14 + p34;
   out_2655578044031403162[12] = -k4*p11 + p41;
   out_2655578044031403162[13] = -k4*p12 + p42;
   out_2655578044031403162[14] = -k4*p13 + p43;
   out_2655578044031403162[15] = -k4*p14 + p44;
}
