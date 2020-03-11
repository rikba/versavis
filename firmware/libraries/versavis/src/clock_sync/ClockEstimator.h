#ifndef CLOCK_SYNC__CLOCKESTIMATOR__H
#define CLOCK_SYNC__CLOCKESTIMATOR__H
void predictX(double dt, double u1, double x1, double x2, double x3, double x4, double *out_856630842665997621);
void predictP(double dt, double p11, double p12, double p13, double p14, double p21, double p22, double p23, double p24, double p31, double p32, double p33, double p34, double p41, double p42, double p43, double p44, double u1, double var_x1, double var_x2, double var_x3, double var_x4, double x3, double x4, double *out_1737487658341360313);
void computeResidual(double x1, double z1, double *out_4198750698365820525);
void computeSInverse(double p11, double var_z1, double *out_4349681001235345741);
void computeK(double p11, double p21, double p31, double p41, double s_inv11, double *out_7426116661966127333);
void estimateX(double k1, double k2, double k3, double k4, double x1, double x2, double x3, double x4, double z1, double *out_5254171509282157043);
void estimateP(double k1, double k2, double k3, double k4, double p11, double p12, double p13, double p14, double p21, double p22, double p23, double p24, double p31, double p32, double p33, double p34, double p41, double p42, double p43, double p44, double *out_4625592411420530874);
#endif
