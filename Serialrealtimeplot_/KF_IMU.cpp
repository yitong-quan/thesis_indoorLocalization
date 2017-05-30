#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include "MatMath.h"

// mm = 15;
// nn = 7;
// ll = 1;
//void Kalman_filter(float X_new[15][1], float P_new[15][15], float X[15][1], float P[15][15], float m[7][1], float H[7][15], float acc_b[3][1], float C_b2n_m[3][3], float Q[15][15], float R[7][7], float dt, int mm, int nn, int ll)
void Kalman_filter(float* X_new, float* P_new, float* X, float* P, float* m, float* H, float* acc_b, float* C_b2n_m, float* Q, float* R, float dt)
{

	float acc_n[3][1];
	MatrixMul((float*) C_b2n_m,  (float*) acc_b, 3, 3, 1, (float*) acc_n);
	float S_acc_n[3][3];
	S_acc_n[0][0] = 0;
    S_acc_n[0][1] = -acc_n[2][0];
	S_acc_n[0][2] = acc_n[1][0];
	S_acc_n[1][0] = acc_n[2][0];
	S_acc_n[1][1] = 0;
	S_acc_n[1][2] = -acc_n[0][0];
    S_acc_n[2][0] = -acc_n[1][0];
    S_acc_n[2][1] = acc_n[0][0];
    S_acc_n[2][2] = 0;
	
	//float H[7][15];
	//zeros(7, 15, (float*)H);
	float A[15][15];
	zeros(15, 15, (float*)A);
	
	
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){
		     if (i==j){
			     A[i][j] = 1;
			     A[i+3][j+3] = 1;
                 A[i+6][j+6] = 1;
				 A[i+9][j+9] = 1;
				 A[i+12][j+12] = 1;
				 A[i+6][j+9] = dt;
				 //H[i][j+2] = 1;
				 //H[i+4][j+9] = 1;
			 }
			 A[i][j+3] = dt*C_b2n_m[3*i+j];
			 A[i+9][j+12] = dt*C_b2n_m[3*i+j];
			 A[i+9][j] = -dt*S_acc_n[i][j];
			 
			 
		}
	}
	//H[0][2] = 1;
	float X_m[15][1];
	MatrixMul((float*) A, (float*) X, 15, 15, 1, (float*) X_m);
	
	float A_TP[15][15]; //transpose of A
    float AP[15][15];
    float APA_TP[15][15];
	MatrixMul3((float*)A, (float*)P, (float*)A_TP, (float*)AP, 15, 15, (float*)APA_TP);
	float P_m[15][15];
	MatrixAdd((float*) APA_TP, (float*) Q, 15, 15, (float*) P_m);
	//////compass yaw/////////////
	/*
	float H_TP[15][7]; //transpose of H
    float HP_m[7][15];
    float HP_mH_TP[7][7];
	MatrixTranspose((float*)H, 7, 15, (float*)H_TP);
	MatrixMul((float*)H, (float*)P_m, 7, 15, 15, (float*)HP_m);
    MatrixMul((float*)HP_m, (float*)H_TP, 7, 15, 7, (float*)HP_mH_TP);
	//MatrixMul3((float*)H, (float*)P_m, (float*)H_TP, (float*)HP_m, 7, 15, (float*)HP_mH_TP);
	float HP_mH_TPaddR[7][7];
	MatrixAdd((float*) HP_mH_TP, (float*) R, 7, 7, (float*) HP_mH_TPaddR);
	float invHP_mH_TPaddR[7][7];
	MatrixInvCD((float*) HP_mH_TPaddR, 7, (float*) invHP_mH_TPaddR);
	float P_mH_TP[15][7];
	MatrixMul((float*) P_m, (float*) H_TP, 15, 15, 7, (float*) P_mH_TP);
	float K[15][7];
	MatrixMul((float*) P_mH_TP, (float*) invHP_mH_TPaddR, 15, 7, 7, (float*) K);
	float eye15[15][15];
	eye(15, 1, (float*)eye15);
	float KH[15][15];
	MatrixMul((float*) K, (float*) H, 15, 7, 15, (float*) KH);
	float eye15subKH[15][15];
	MatrixSub((float*) eye15, (float*) KH, 15, 15, (float*) eye15subKH);
	float eye15subKH_TP[15][15]; //transpose of H
    float esKHP_m[15][15];
    float esKHP_mesKH_TP[15][15];
	MatrixMul3((float*)eye15subKH, (float*)P_m, (float*)eye15subKH_TP, (float*)esKHP_m, 15, 15, (float*)esKHP_mesKH_TP);
	MatrixAdd((float*) esKHP_mesKH_TP, (float*) Q, 15, 15, (float*) P_new);
	float HX_m[7][1];
	MatrixMul((float*) H, (float*) X_m, 7, 15, 1, (float*) HX_m);
	float msubHX_m[7][1];
	MatrixSub((float*) m, (float*) HX_m, 7, 1, (float*) msubHX_m);
	float KmsubHX_m[15][1];
	MatrixMul((float*) K, (float*) msubHX_m, 15, 7, 1, (float*) KmsubHX_m);
	MatrixAdd((float*) X_m, (float*) KmsubHX_m, 15, 1, (float*) X_new);
	*/
	//////compass 3D/////////////
	float H_TP[15][9]; //transpose of H
    float HP_m[9][15];
    float HP_mH_TP[9][9];
	MatrixTranspose((float*)H, 9, 15, (float*)H_TP);
	MatrixMul((float*)H, (float*)P_m, 9, 15, 15, (float*)HP_m);
    MatrixMul((float*)HP_m, (float*)H_TP, 9, 15, 9, (float*)HP_mH_TP);
	//MatrixMul3((float*)H, (float*)P_m, (float*)H_TP, (float*)HP_m, 7, 15, (float*)HP_mH_TP);
	float HP_mH_TPaddR[9][9];
	MatrixAdd((float*) HP_mH_TP, (float*) R, 9, 9, (float*) HP_mH_TPaddR);
	float invHP_mH_TPaddR[9][9];
	MatrixInvCD((float*) HP_mH_TPaddR, 9, (float*) invHP_mH_TPaddR);
	float P_mH_TP[15][9];
	MatrixMul((float*) P_m, (float*) H_TP, 15, 15, 9, (float*) P_mH_TP);
	float K[15][9];
	MatrixMul((float*) P_mH_TP, (float*) invHP_mH_TPaddR, 15, 9, 9, (float*) K);
	float eye15[15][15];
	eye(15, 1, (float*)eye15);
	float KH[15][15];
	MatrixMul((float*) K, (float*) H, 15, 9, 15, (float*) KH);
	float eye15subKH[15][15];
	MatrixSub((float*) eye15, (float*) KH, 15, 15, (float*) eye15subKH);
	float eye15subKH_TP[15][15]; //transpose of H
    float esKHP_m[15][15];
    float esKHP_mesKH_TP[15][15];
	MatrixMul3((float*)eye15subKH, (float*)P_m, (float*)eye15subKH_TP, (float*)esKHP_m, 15, 15, (float*)esKHP_mesKH_TP);
	MatrixAdd((float*) esKHP_mesKH_TP, (float*) Q, 15, 15, (float*) P_new);
	float HX_m[9][1];
	MatrixMul((float*) H, (float*) X_m, 9, 15, 1, (float*) HX_m);
	float msubHX_m[9][1];
	MatrixSub((float*) m, (float*) HX_m, 9, 1, (float*) msubHX_m);
	float KmsubHX_m[15][1];
	MatrixMul((float*) K, (float*) msubHX_m, 15, 9, 1, (float*) KmsubHX_m);
	MatrixAdd((float*) X_m, (float*) KmsubHX_m, 15, 1, (float*) X_new);
	///////////////////////////////////////

}