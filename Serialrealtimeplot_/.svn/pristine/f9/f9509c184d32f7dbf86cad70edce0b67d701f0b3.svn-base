#ifndef QRIEN_MAG
#define QRIEN_MAG

void DCM_calcu(float output[3][1], float DCM[3][3]);
float norm(int num, float* A);
float mean(int num, float* A);
void ZUPT(float th, float sn, float W_old, float W_new, int nnn_old, float output[3], int* nnn_new, float output_new[3]);
void orien_mag_std(float acc[3], float mag[3], float EUL[3]);
void mean_adv(float tha, float sna, float W_old, float W_new, int mmmm_old, float data[3], float data_old[3], int* mmmm_new, float data_new[3]);
void orien_mag_adv(int counter, float acc[3], float output[3], float tha, float sna, float W_old, float W_new, float EUL[3], int mmmm_old, float HmeasB_old[3], int* mmmm_new, float HmeasB_new[3], float* yaw_B);
void orien_mag_calcu(float Magfini, float Hthini, float Mthini, float Magf, float Hth, float Mth, float yaw_B, float yaw_M, float* YAW);
void Mag_Mth_ini_calcu(float* mag, float* acc, int m, float* Magfini, float* Mthini);




#endif
