#include <stdio.h>
#include <stdarg.h>
//#include <math.h>
#include <cmath>
#include "MatMath.h"

void DCM_calcu(float output[3][1], float DCM[3][3])
{
    float s0=sin(output[0][0]);
    float c0=cos(output[0][0]);
    float s1=sin(output[1][0]);
    float c1=cos(output[1][0]);
    float s2=sin(output[2][0]);
    float c2=cos(output[2][0]);
  
    DCM[0][0] = c1*c2;
	DCM[1][0] = s2*c1;
	DCM[2][0] = -s1;
	DCM[0][1] = -c0*s2+s0*s1*c2;
	DCM[1][1] = c0*c2+s0*s1*s2;
	DCM[2][1] = s0*c1;
	DCM[0][2] = s0*s2+c0*s1*c2;
	DCM[1][2] = -s0*c2+c0*s1*s2;
	DCM[2][2] = c0*c1;
}

float norm(int num, float* A)
{
   float a = 0;
   for (int i=0; i<num; i++){
        a = a + pow(A[i],2);
   }
   a = sqrt(a);
   return a;
}

float mean(int num, float* A)
{
   float a = 0;
   for (int i=0; i<num; i++){
        a = a + A[i];
   }
   a = a/num;
   return a;
}
/*
float abs(float a)
{
    if(a<0){
	   a = -a;}
	return a;

}
*/
void ZUPT(float th, float sn, float W_old, float W_new, int nnn_old, float output[3], int* nnn_new, float output_new[3])
{
    if ((W_new<th)&&(W_old<th)){
        *nnn_new = nnn_old + 1;
	    if (*nnn_new >= sn){
		    output[0] = 0;
		    output[1] = 0;
            output[2] = 0;
		} 
	}
	if ((W_new>th)&&(W_old<th)){
		*nnn_new = 0; 
	}
    output_new[0] = output[0];
    output_new[1] = output[1];
    output_new[2] = output[2];
}
void orien_mag_std(float acc[3], float mag[3], float EUL[3])
{
    float Xh, Yh;
	EUL[0] = atan2(acc[1],acc[2]);
	EUL[1] = asin(acc[0]/(-norm(3,(float*)acc)));
    Xh = mag[0]*cos(EUL[1])+mag[1]*sin(EUL[0])*sin(EUL[1])+mag[2]*cos(EUL[0])*sin(EUL[1]);
    Yh = mag[1]*cos(EUL[0])-mag[2]*sin(EUL[0]);
    EUL[2] = -atan2(Yh,Xh);
}

void mean_adv(float tha, float sna, float W_old, float W_new, int mmmm_old, float data[3], float data_old[3], int* mmmm_new, float data_new[3])
{
  
	if ((W_new<tha)&&(W_old<tha)){
        *mmmm_new = mmmm_old + 1;
	    if (*mmmm_new >= sna){
		    data[0] = (data[0] + data_old[0] * (*mmmm_new-1))/ (*mmmm_new);
		    data[1] = (data[1] + data_old[1] * (*mmmm_new-1))/ (*mmmm_new);
            data[2] = (data[2] + data_old[2] * (*mmmm_new-1))/ (*mmmm_new);
		} 
	}
	if ((W_new>tha)&&(W_old<tha)){
		*mmmm_new = 0; 
	}
    data_new[0] = data[0];
    data_new[1] = data[1];
    data_new[2] = data[2];
}
// mmmm_old = mmmm_new
// data_old = data_new
// W_old = W_new

void orien_mag_adv(int counter, float acc[3], float output[3][1], float tha, float sna, float W_old, float W_new, float EUL[3], int mmmm_old, float HmeasB_old[3], int* mmmm_new, float HmeasB_new[3], float* yaw_B)
{
	float HmeasB[3][1];
	float DCM[3][3];
	//float W_new;
    //W_new = norm(3, (float*)mag);
	DCM_calcu(output, DCM);
	float B[3][1] = {1,
		              0,
				      0};
	MatrixMul((float*) DCM, (float*) B, 3, 3, 1, (float*)HmeasB); 
	if (counter>1){
	    if ((W_new<tha)&&(W_old<tha)){
             *mmmm_new = mmmm_old + 1;
	         if (*mmmm_new >= sna){
		         HmeasB[0][0] = (HmeasB[0][0] + HmeasB_old[0] * (*mmmm_new-1))/ (*mmmm_new);
		         HmeasB[1][0] = (HmeasB[1][0] + HmeasB_old[1] * (*mmmm_new-1))/ (*mmmm_new);
                 HmeasB[2][0] = (HmeasB[2][0] + HmeasB_old[2] * (*mmmm_new-1))/ (*mmmm_new);
			 } 
		}
	    if ((W_new>tha)&&(W_old<tha)){
		     *mmmm_new = 0;
		}
	}
    HmeasB_new[0] = HmeasB[0][0];
    HmeasB_new[1] = HmeasB[1][0];
    HmeasB_new[2] = HmeasB[2][0];
 
	float XhmB, YhmB;
    XhmB = HmeasB_new[0]*cos(EUL[1])+HmeasB_new[1]*sin(EUL[0])*sin(EUL[1])+HmeasB_new[2]*cos(EUL[0])*sin(EUL[1]);
    YhmB = HmeasB_new[1]*cos(EUL[0])-HmeasB_new[2]*sin(EUL[0]);
    *yaw_B = -atan2(YhmB,XhmB);
} 

// mmmm_old = mmmm_new
// HmeasB_old = Hmeas_new
// W_old = W_new

void orien_mag_calcu(float Magfini, float Hthini, float Mthini, float Magf, float Hth, float Mth, float yaw_B, float yaw_M, float* YAW)
{
	*YAW = yaw_B;
	if (Magfini<1.1 && Magfini>0.9 && fabs(Hth-Hthini)>fabs(Mth-Mthini) && Magf<Magfini+0.1 && Magf>Magfini-0.1){
        *YAW = yaw_M;
	}
}

void Mag_Mth_ini_calcu(float* mag, float* acc, int m, float* Magfini, float* Mthini)
{ 
	float a = 0;
	float b = 0;
	for (int i=0; i<m; i++){
		a = a + sqrt(pow(mag[0+i],2)+pow(mag[1+i],2)+pow(mag[2+i],2));
		b = b + mag[0+i]*acc[0+i] + mag[1+i]*acc[1+i] + mag[2+i]*acc[2+i];
	}
    *Magfini = a;
	*Mthini = b;
}
