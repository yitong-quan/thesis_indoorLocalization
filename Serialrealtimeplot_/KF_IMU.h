#ifndef KF_IMU
#define KF_IMU

//void Kalman_filter(float X_new[15][1], float P_new[15][15], float X[15][1], float P[15][15], float m[7][1], float H[7][15], float acc_b[3][1], float C_b2n_m[3][3], float Q[15][15], float R[7][7], float dt);
void Kalman_filter(float* X_new, float* P_new, float* X, float* P, float* m, float* H, float* acc_b, float* C_b2n_m, float* Q, float* R, float dt);

#endif