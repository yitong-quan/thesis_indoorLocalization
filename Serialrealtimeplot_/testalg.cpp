#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define _CRT_SECURE_NO_DEPRECATE
//#pragma warning (disable : 4996)


//float Acc[N][3];
//float Gyr[N][3];
//float Accf[N];
//float Gyrf[N];
//float tim[N];
//float freq[N];

void testalg(float* Acc, float* Gyr, float* Accf, float* Gyrf, float* tim, float* freq)
{
    ///////////// Read the sensor data ///////////////////
    float d;
    float A[100][9];
    int row = 0;
    int col = 0;

    FILE* f = fopen("C:\\Users\\Zhang\\Documents\\QTcreator project\\readtextplot\\testdatanew.txt", "r");
    while (!feof(f)) {
        fscanf(f, "%f", &d);
        A[row][col] = d;
        col++;
        if (col == 9) {
            row++;
            col = 0;
        }
    }
    fclose(f);

    float Fs = 35.2;
    float dt = 1/Fs; //sampling time step
    ///////////// save the sensor data ////////////////////
    for (int i = 0; i < 100; i++){
        for (int j = 3; j < 6; j++){
            Acc[i*3+(j - 3)] = A[i][j];
            Gyr[i*3+(j - 3)] = A[i][j + 3];
        }
        tim[i] = i*dt;
        freq[i] = i*Fs / 2 / 100;
        Accf[i] = sqrt(pow(Acc[i*3],2)+pow(Acc[i*3+1],2)+pow(Acc[i*3+2],2));
        Gyrf[i] = sqrt(pow(Gyr[i*3],2)+pow(Gyr[i*3+1],2)+pow(Gyr[i*3+2],2));
    }
}
