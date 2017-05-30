
#ifndef MATMATH
#define MATMATH

int CholDec(float *A, int n,float *chol);
void cholsl(float *A,int n, float p[], float b[], float x[]);
void lubksb(float *A, int n, int *indx, float b[]);
int MatrixInvCD(float *A, int n, float *AInverse);
int MatrixInvLU(float *A, int n,float *AInverse);
int MatrixInvGJ(float* A, int n, float* AInverse);
void MatrixMul(float* A, float* B, int m, int p, int n, float* C);
void MatrixAdd(float* A, float* B, int m, int n, float* C);
void MatrixSub(float* A, float* B, int m, int n, float* C);
void MatrixTranspose(float* A, int m, int n, float* C);
float max_array(float a[], int num_elements);
float trace(int num, float* A);
void zero( int num, float* A);
void zeros( int num1, int num2, float* A);
void eye( int num, float a, float* A);
void MatrixMul3(float* A, float* B, float* A_TP, float* AB, int m, int n, float* C);
#endif
