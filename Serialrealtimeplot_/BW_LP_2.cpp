
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>




//x[2] = x;  put sensor data at this time period

void Butterworth_LP_2(float* data_in, float* data_out, int f_cutoff, int n)
{
   double a[3], b[3];
   double y_new;
   double x[3]; 
   double y[2]; 
   a[0] = 1;
   double zi[2];
   
   switch (f_cutoff) {

   case  1:
	          a[1] = -1.561018075800718;            
		      a[2] = 0.641351538057563;
			  b[0] = 0.020083365564211;
			  b[1] = 0.040166731128423;
              b[2] = b[0];
              zi[0] = 0.979916634435788;
              zi[1] = -0.621268172493351;
			  break;
   case  5:
	          a[1] = -3.3307e-16;            
		      a[2] = 0.171572875253810;
			  b[0] = 0.292893218813452;
			  b[1] = 0.585786437626905;
              b[2] = b[0];
              zi[0] = 0.707106781186548;
              zi[1] = 0.121320343559643;
			  break;
   case  8:
	          a[1] = 1.142980502539901;            
		      a[2] = 0.412801598096189;
			  b[0] = 0.638945525159022;
			  b[1] = 1.277891050318045;
              b[2] = b[0];
              zi[0] = 0.361054474840978;
              zi[1] = 0.226143927062834;
			  break;
   case  9:
	          a[1] = 1.561018075800718;            
		      a[2] = 0.641351538057563;
			  b[0] = 0.800592403464570;
			  b[1] = 1.601184806929141;
              b[2] = b[0];
              zi[0] = 0.199407596535430;
              zi[1] = 0.159240865407007;
			  break;
   default:
	          a[1] = 0;            
		      a[2] = 0;
			  b[0] = 1;
			  b[1] = 0;
			  b[2] = 0;
              zi[0] = 0;
              zi[1] = 0;
	          break;
   }
   
   for (int ii=0; ii<n; ii++){
         x[2] = data_in[ii];
		 if (ii==0){ 
		     y_new = b[0]*x[2] + zi[0]*a[0]*data_in[0];
             y[1] = y_new;
             x[1] = x[2];
             data_out[ii] = y_new;
		 }
	     else if (ii==1){ 
		          y_new = b[0]*x[2] + b[1]*x[1] + zi[1]*a[0]*data_in[0] - a[1]*y[1];
                  y[0] = y[1];
                  y[1] = y_new;
                  x[0] = x[1];
                  x[1] = x[2];
                  data_out[ii] = y_new;
		 }
		 else {
         y_new = b[0]*x[2] + b[1]*x[1] + b[2]*x[0] - a[1]*y[1] - a[2]*y[0];
         y[0] = y[1];
         y[1] = y_new;
         x[0] = x[1];
         x[1] = x[2];
		 data_out[ii] = y_new;
         }
	}
}

void filter_2_LP(float* aa, float* d, int f_cutoff, int len){
	 int nfact = 6; //3*((size of max(size of a or b))-1)
     int n = 2*nfact + len; 
     float *a = (float*)malloc(n*sizeof(float));	 
	 float *b = (float*)malloc(n*sizeof(float));
	 float *bb = (float*)malloc(n*sizeof(float));
     float *c = (float*)malloc(n*sizeof(float));
     float *cc = (float*)malloc(n*sizeof(float));
     //float *d = (float*)malloc(len*sizeof(float));
	
	 for (int i=0; i<nfact; i++){
		  a[i] = 2*aa[0] - aa[nfact-i];
	 }
	 for (int j=nfact; j<nfact+len; j++){
		  a[j] = aa[j-nfact];
	 }
     for (int k=nfact+len; k<n; k++){
	      a[k] = 2*aa[len-1] - aa[len-2-(k-nfact-len)];
	 }

     Butterworth_LP_2((float*) a, (float*) b, f_cutoff, n);

     for (int jj=0; jj<n; jj++){
		  bb[jj] = b[n-jj-1];
	 }  
         
     Butterworth_LP_2((float*) bb, (float*) c, f_cutoff, n);
    
     for (int kk=0; kk<n; kk++){
		  cc[kk] = c[n-kk-1];
	 } 
	
	 for (int ll=0; ll<len; ll++){
		  d[ll] = cc[ll+nfact];
	 }
		       
	 free(a);
	 free(b);
	 free(c);
	 free(bb);
     
}

void LP_2(int counter, float* x, int i, float y, float* d, int start, int* start_new, int* ena, int f_cutoff, int len) 
{
	         if(counter <= len)
			 {
			   //counter++;
			    x[i] = y;
		        x[i+len] = y;
			    if(counter == len){
			      filter_2_LP(&x[start], d, f_cutoff, len);
				  *ena=1;
                  //printf("%10.18f  %10.18f  %10.18f  %10.18f  %10.18f  %10.18f  %10.18f  %10.18f  %10.18f  %10.18f\n", d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[9]);
				}
			 }
             else 
			 {			
			    x[start] = y;
                x[start+len] = y;
                start = (start+1)%len;
			    *start_new = start;
				if (start==0){
					//float med[len];
					for (int kkk=0; kkk<len; kkk++){
						//med[kkk]=x[kkk];
						x[kkk]=x[kkk+len];
					}
				}
                filter_2_LP(&x[start], d, f_cutoff, len);
                *ena=1;
				
            //printf("%10.18f  %10.18f  %10.18f  %10.18f  %10.18f  %10.18f  %10.18f  %10.18f  %10.18f  %10.18f\n", d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[9]);
			 }  
}