
//******************************************************************************
// Matrix Math functions
//******************************************************************************

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define TINY 1.0e-20

int *ivector(long nl, long nh){
  int *v;
  v=(int *)malloc((size_t) ((nh-nl+2)*sizeof(int)));
  //if (!v) printf("allocation failure in ivector()");
  return v-nl+1;
}

/********************************************************/
float *vector(long nl, long nh){
  float *v;
  v=(float *)malloc((size_t) ((nh-nl+2)*sizeof(float)));
  return v-nl+1;
}

/********************************************************/
void free_vector(float *v, long nl, long nh){
  free((char*) (v+nl-1));
}

/********************************************************/
void free_ivector(int *v, long nl, long nh){
  free((char*) (v+nl-1));
}

//------------------------------------------------------------------------------
// This module computes Cholesky Decomposition (lower-triangular Cholesky Factor)
//------------------------------------------------------------------------------
int CholDec(float *A, int n,float *chol){
// A = input matrix (n x n) "A does not ruin during the process"
// n = dimension of A 
// chol = lower-triangular Cholesky Factor (n x n)
// This function returns the lower-triangular Cholesky Factor.
// The function returns 1 on success, 0 on failure.  
  int i,j,k;
  float sum,*p;
  
  float* AOrig = (float*)calloc(n*n, sizeof(float));
  p = vector(1,n);
  // Matrecies Copy
  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++){
      AOrig[n*i+j] = A[n*i+j];
      chol[n*i+j] = 0;
    }
  
  for(i=0;i<n;i++){
    for(j=0;j<n;j++){
      for(sum=AOrig[n*i+j],k=i-1;k>=0;k--) sum -= AOrig[n*i+k]*AOrig[n*j+k];
      if(i == j){
        if(sum <= 0.0){
          free_vector(p,1,n);
          free(AOrig);
          return 0;
        }
        p[i] = sqrt(sum);
      } else AOrig[n*j+i]=sum/p[i];
    }
  }
  
  for(i=0;i<n;i++){
    for(j=0;j<n;j++){
      chol[n*i+j]=((i>j) ? AOrig[n*i+j] : (i == j ? p[i] : 0.0));
      if (i>j) chol[n*i+j]=AOrig[n*i+j];
      else chol[n*i+j]=(i == j ? p[i] : 0.0);
    }
  }
  free_vector(p,1,n);
  free(AOrig);
  return 1;
}

/********************************************************/
void cholsl(float *A,int n, float p[], float b[], float x[]){
  int i,k;
  float sum;
  
  // Solve L.y=b storing y in x
  for (i=0;i<n;i++){
    for (sum=b[i],k=i-1;k>=0;k--) sum -= A[n*i+k]*x[k];
    x[i] = sum/p[i];
  }
  
  // Solve L'.x=y
  for(i=n-1;i>=0;i--) {
    for (sum=x[i],k=i+1;k<n;k++) sum -= A[n*k+i]*x[k];
    x[i]=sum/p[i];
  }
}

/********************************************************/
void lubksb(float *A, int n, int *indx, float b[]){
  int i,ii=-1,ip,j;
  float sum;
  
  // Forward Substitution
  for (i=0;i<n;i++){
    ip=indx[i];
    sum=b[ip];
    b[ip]=b[i];
    if (ii>=0)
        for (j=ii;j<=i-1;j++) sum -= A[n*i+j]*b[j];
    else if (sum) ii=i;
    b[i]=sum;
  }
  // BackSubstitution
  for(i=n-1;i>=0;i--){
    sum=b[i];
    for (j=i+1;j<n;j++) sum -= A[n*i+j]*b[j];
    b[i]=sum/A[n*i+i];
  }
}

//------------------------------------------------------------------------------
// This module computes Matrix inversion using Cholesky Decomposition
//------------------------------------------------------------------------------
int MatrixInvCD(float *A, int n, float *AInverse){
// A = input matrix (n x n) "A does not ruin during the process"
// n = dimension of A 
// AInverse = inverted matrix (n x n)
// This function inverts a matrix based on the Cholesky Decomposition method 
// "works only with symmetric definite positive matrix"
// The function returns 1 on success, 0 on failure.  
  int i,j,k;
  float sum, *col, *p, *x;
  col = vector(1,n);  p = vector(1,n);  x = vector(1,n);  
  float* AOrig = (float*)calloc(n*n, sizeof(float));

  // Matrecies Copy
  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++){
      AOrig[n*i+j] = A[n*i+j];
      AInverse[n*i+j] = 0;
    }  
  
  for(i=0;i<n;i++){
    for(j=i;j<n;j++){
      for(sum=AOrig[n*i+j],k=i-1;k>=0;k--) sum -= AOrig[n*i+k]*AOrig[n*j+k];
      if(i == j){
        if(sum <= 0.0){
          free_vector(p,1,n);
          free_vector(col,1,n);
          free_vector(x,1,n);
          free(AOrig);
          return 0;       
        }
        p[i] = sqrt(sum);
      } else AOrig[n*j+i]=sum/p[i];
    }
  }
  
  for(j=0;j<n;j++){
    for(i=0;i<n;i++) col[i]=0.0;
    col[j]=1.0;
    cholsl(AOrig,n,p,col,x);
    for(i=0;i<n;i++) AInverse[n*i+j]=x[i];
  }
  free_vector(p,1,n);
  free_vector(col,1,n);
  free_vector(x,1,n);
  free(AOrig);
  return 1;
}


//------------------------------------------------------------------------------
// This module computes Matrix inversion using LU Decomposition
//------------------------------------------------------------------------------
int MatrixInvLU(float *A, int n,float *AInverse){
// A = input matrix (n x n) "A does not ruin during the process"
// n = dimension of A 
// AInverse = inverted matrix (n x n)
// This function inverts a matrix based on the LU Decomposition method with partial pivoting.
// The function returns 1 on success, 0 on failure.  
  int i,imax,j,k;
  float big,dum,sum,temp;
  float *vv,*col;
  int *indx;
  
  float* AOrig = (float*)calloc(n*n, sizeof(float));
  indx = ivector(1,n); col = vector(1,n); vv = vector(1,n);
  // Matrecies Copy
  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++){
      AOrig[n*i+j] = A[n*i+j];
      AInverse[n*i+j] = 0;
    }
  // Getting the Scaling factor
  for (i=0;i<n;i++){
    big=0.0;
    for (j=0;j<n;j++)
      if ((temp=fabs(A[n*i+j]))>big) big=temp;
    if (big == 0.0){
      free_ivector(indx,1,n);
      free_vector(col,1,n);
      free_vector(vv,1,n);
      free(AOrig);
      return 0;
      }
    vv[i] = 1.0/big;
  }
  
  /* LU Decomposition Process*/
 
  for (j=0;j<n;j++){
      // Calculate for 'i<j'
      for (i=0;i<j;i++){
        sum=AOrig[n*i+j];
        for (k=0;k<i;k++) sum -= AOrig[n*i+k]*AOrig[n*k+j];
        AOrig[n*i+j]=sum;
      }
      // Calculate for 'i=j' and 'i>j'
      big=0.0;
      for (i=j;i< n;i++){
        sum=AOrig[n*i+j];
        for (k=0;k<j;k++) sum -= AOrig[n*i+k]*AOrig[n*k+j];
        AOrig[n*i+j]=sum;
        if ((dum=vv[i]*fabs(sum)) >= big) {
          big = dum;
          imax = i;
        }
      }
    
      // Interchange rows between 'imax' and 'j'
      if (j != imax){
        for (k=0;k<n;k++){
          dum=AOrig[n*imax+k];
          AOrig[n*imax+k]=AOrig[n*j+k];
          AOrig[n*j+k]=dum;
        }
        vv[imax]=vv[j];
      }
      indx[j]=imax;
    
      // Replace Zero by Tiny value
      if (AOrig[n*j+j]==0.0) AOrig[n*j+j]=TINY;
    
      // Division by Pivot
      if (j != n){
        dum=1.0/(AOrig[n*j+j]);
        for (i=j+1;i<n;i++) AOrig[n*i+j] *= dum;
      }
   }
  
  /* Calculate the matrix inverse*/

  for(j=0;j<n;j++){
      for(i=0;i<n;i++) col[i]=0.0;
      col[j]=1.0;
      lubksb(AOrig,n,indx,col);  
      for(i=0;i<n;i++) AInverse[n*i+j]=col[i];
    }
  free_ivector(indx,1,n);
  free_vector(col,1,n);
  free_vector(vv,1,n);
  free(AOrig);
  return 1;
}

//------------------------------------------------------------------------------
// This module computes Matrix inversion using Gauss Jordan Method
//------------------------------------------------------------------------------

int MatrixInvGJ(float* A, int n, float* AInverse){
// A = input matrix (n x n) "A does not ruin during the process"
// n = dimension of A 
// AInverse = inverted matrix (n x n)
// This function inverts a matrix based on the Gauss Jordan method.
// The function returns 1 on success, 0 on failure.
int i, j, iPass, imx, icol, irow;
float det, temp, pivot, factor;
float* AOrig = (float*)calloc(n*n, sizeof(float));
det = 1;
for (i = 0; i < n; i++)
	{
	for (j = 0; j < n; j++)
		{
                AOrig[n*i+j] = A[n*i+j];
		AInverse[n*i+j] = 0;
		}
	AInverse[n*i+i] = 1;
	}
// The current pivot row is iPass.  
// For each pass, first find the maximum element in the pivot column.
for (iPass = 0; iPass < n; iPass++)
	{
	imx = iPass;
	for (irow = iPass; irow < n; irow++)
		{
		if (fabs(AOrig[n*irow+iPass]) > fabs(AOrig[n*imx+iPass])) imx = irow;
		}
	// Interchange the elements of row iPass and row imx in both AOrig and AInverse.
	if (imx != iPass)
		{
		for (icol = 0; icol < n; icol++)
			{
			temp = AInverse[n*iPass+icol];
			AInverse[n*iPass+icol] = AInverse[n*imx+icol];
			AInverse[n*imx+icol] = temp;
			if (icol >= iPass)
				{
				temp = AOrig[n*iPass+icol];
				AOrig[n*iPass+icol] = AOrig[n*imx+icol];
				AOrig[n*imx+icol] = temp;
				}
			}
		}
	// The current pivot is now AOrig[iPass][iPass].
	// The determinant is the product of the pivot elements.
	pivot = AOrig[n*iPass+iPass];
	det = det * pivot;
	if (det == 0) 
		{
		free(AOrig);
		return 0;
		}
	for (icol = 0; icol < n; icol++)
		{
		// Normalize the pivot row by dividing by the pivot element.
		AInverse[n*iPass+icol] = AInverse[n*iPass+icol] / pivot;
		if (icol >= iPass) AOrig[n*iPass+icol] = AOrig[n*iPass+icol] / pivot;
		}
	for (irow = 0; irow < n; irow++)
		// Add a multiple of the pivot row to each row.  The multiple factor 
		// is chosen so that the element of AOrig on the pivot column is 0.
		{
		if (irow != iPass) factor = AOrig[n*irow+iPass];
		for (icol = 0; icol < n; icol++)
			{
			if (irow != iPass)
				{
				AInverse[n*irow+icol] -= factor * AInverse[n*iPass+icol];
				AOrig[n*irow+icol] -= factor * AOrig[n*iPass+icol];
				}
			}
		}
	}
	free(AOrig);
	return 1;
}

//------------------------------------------------------------------------------
// This module computes Matrix Multiplication
//------------------------------------------------------------------------------
void MatrixMul(float* A, float* B, int m, int p, int n, float* C)
// Matrix Multiplication Routine
{
    // A = input matrix (m x p) "A does not ruin during the process"
    // B = input matrix (p x n) "B does not ruin during the process"
    // m = number of rows in A
    // p = number of columns in A = number of rows in B
    // n = number of columns in B
    // C = output matrix = A*B (m x n)
    int i, j, k;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
        {
          C[n*i+j]=0;
          for (k=0;k<p;k++)
            C[n*i+j]= C[n*i+j]+A[p*i+k]*B[n*k+j];
        }
}

//------------------------------------------------------------------------------
// This module computes Matrix Addition
//------------------------------------------------------------------------------
void MatrixAdd(float* A, float* B, int m, int n, float* C)
// Matrix Addition Routine
{
    // A = input matrix (m x n) "A does not ruin during the process"
    // B = input matrix (m x n) "B does not ruin during the process"
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A+B (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j]+B[n*i+j];
}

//------------------------------------------------------------------------------
// This module computes Matrix Subtraction
//------------------------------------------------------------------------------
void MatrixSub(float* A, float* B, int m, int n, float* C)
// Matrix Subtraction Routine
{
    // A = input matrix (m x n) "A does not ruin during the process"
    // B = input matrix (m x n) "B does not ruin during the process"
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A-B (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j]-B[n*i+j];
}

//------------------------------------------------------------------------------
// This module computes Matrix Transpose
//------------------------------------------------------------------------------
void MatrixTranspose(float* A, int m, int n, float* C)
// Matrix Transpose Routine
{
    // A = input matrix (m x n) "A does not ruin during the process"
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = the transpose of A (n x m)
  int i, j;

  for (i=0;i<m;i++)
    for(j=0;j<n;j++)
      C[m*j+i]=A[n*i+j];
  
}
//------------------------------------------------------------------------------
// This module computes Maximum Value in an Array
//------------------------------------------------------------------------------
float max_array(float a[], int num_elements)
{
   int i;
   float max=-32000;
   for (i=0; i<num_elements; i++)
   {
	 if (a[i]>max)
	 {
	    max=a[i];
	 }
   }
   return(max);
}
//------------------------------------------------------------------------------
// This module computes Trace of Matrix
//------------------------------------------------------------------------------
float trace(int num, float* A)
{
   float a = 0;
   for (int i=0; i<num; i++){
        a = a + A[i*num+i];
   }
   return a;
}
//------------------------------------------------------------------------------
// This module generates Zero Matrix
//------------------------------------------------------------------------------
void zero( int num, float* A)
{
   for (int i=0; i<num; i++){
        for (int j=0; j<num; j++){
             A[i*num+j] = 0;
        }
   }
}
//------------------------------------------------------------------------------
// This module generates Zeros Matrix
//------------------------------------------------------------------------------
void zeros( int num1, int num2, float* A)
{
   for (int i=0; i<num1; i++){
        for (int j=0; j<num2; j++){
             A[i*num1+j] = 0;
        }
   }
}
//------------------------------------------------------------------------------
// This module generates Eye Matrix with value a
//------------------------------------------------------------------------------
void eye( int num, float a, float* A)
{
   zero( num, A);
   for (int i=0; i<num; i++){
        A[i*num+i] = a;
   }
}
//------------------------------------------------------------------------------
// This module computes Multiplication of 3 Matrice
//------------------------------------------------------------------------------
void MatrixMul3(float* A, float* B, float* A_TP, float* AB, int m, int n, float* C)
{
   MatrixTranspose((float*)A, m, n, (float*)A_TP);
   MatrixMul((float*)A, (float*)B, m, n, n, (float*)AB);
   MatrixMul((float*)AB, (float*)A_TP, m, n, m, (float*)C);
}
