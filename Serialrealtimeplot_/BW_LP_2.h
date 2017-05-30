#ifndef BW_LP_2
#define BW_LP_2

void Butterworth_LP_2(float* data_in, float* data_out, int f_cutoff, int n);

void filter_2_LP(float* aa, float* d, int f_cutoff, int len);
void LP_2(int counter, float* x, int i, float y, float* d, int start, int* start_new, int* ena, int f_cutoff, int len);

#endif
