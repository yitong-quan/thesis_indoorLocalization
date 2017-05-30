#include <stdio.h>
#include <stdarg.h>
#include <math.h>


float AngCal(float A)
{
  float pi = 4*atan(1.0);
  if (A<-pi)
      A=A+2*pi;
  if (A>pi)
	  A=A-2*pi;
  return A;
}



void EULTOQUA(float Eul[3], float Qua[4])
{
  float c2=cos(Eul[2]/2);
  float s2=sin(Eul[2]/2);
  float c1=cos(Eul[1]/2);
  float s1=sin(Eul[1]/2);
  float c0=cos(Eul[0]/2);
  float s0=sin(Eul[0]/2);

  Qua[0] = c2*c1*c0+s2*s1*s0;
  Qua[1] = c2*c1*s0-s2*s1*c0;
  Qua[2] = c2*s1*c0+s2*c1*s0;
  Qua[3] = -c2*s1*s0+s2*c1*c0;
}

void find_position_solo(float output[3][3], float Qua_new[4], float Qua[4], float data[1][7], float current[3][3], float dt)
{
  float s00=sin(current[0][0]);
  float c00=cos(current[0][0]);
  float s01=sin(current[0][1]);
  float c01=cos(current[0][1]);

  //float current[3][3];

  //current[0][0] = initial[0][0];
  //current[0][1] = initial[0][1];
  //current[0][2] = initial[0][2];
  //current[1][0] = initial[1][0];
  //current[1][1] = initial[1][1];
  //current[1][2] = initial[1][2];
  //current[2][0] = initial[2][0];
  //current[2][1] = initial[2][1];
  //current[2][2] = initial[2][2];

  float change[3][3];
  float change_m[3][3];
  float g = 9.81;
  float Qua_n[4];
  float Qua_m[4];
  float norm_qua;
  float M;
  //float earth[3];
  float pi = 4*atan(1.0);

  Qua_n[0] = (-0.5)*(Qua[1]*data[0][4]+Qua[2]*data[0][5]+Qua[3]*data[0][6]);
  Qua_n[1] = (0.5)*(Qua[0]*data[0][4]+Qua[2]*data[0][6]-Qua[3]*data[0][5]);
  Qua_n[2] = (0.5)*(Qua[0]*data[0][5]+Qua[3]*data[0][4]-Qua[1]*data[0][6]);
  Qua_n[3] = (0.5)*(Qua[0]*data[0][6]+Qua[1]*data[0][5]-Qua[2]*data[0][4]);

  Qua_m[0] = dt*Qua_n[0]+Qua[0];
  Qua_m[1] = dt*Qua_n[1]+Qua[1];
  Qua_m[2] = dt*Qua_n[2]+Qua[2];
  Qua_m[3] = dt*Qua_n[3]+Qua[3];
  norm_qua = sqrt(pow(Qua_m[0],2)+pow(Qua_m[1],2)+pow(Qua_m[2],2)+pow(Qua_m[3],2));
  Qua_m[0] = Qua_m[0]/norm_qua;
  Qua_m[1] = Qua_m[1]/norm_qua;
  Qua_m[2] = Qua_m[2]/norm_qua;
  Qua_m[3] = Qua_m[3]/norm_qua;
  M = (-2)*(Qua_m[1]*Qua_m[3]-Qua_m[0]*Qua_m[2]);
  if(M >1){
     M =1;
  }
  if(M <-1){
     M =-1;
  }
  change_m[0][1] = asin(M);
  change_m[0][0] = atan2((2*(Qua_m[0]*Qua_m[1]+Qua_m[2]*Qua_m[3])),(1-2*(pow(Qua_m[1],2)+pow(Qua_m[2],2))));
  change_m[0][2] = atan2((2*(Qua_m[0]*Qua_m[3]+Qua_m[2]*Qua_m[1])),(1-2*(pow(Qua_m[2],2)+pow(Qua_m[3],2))));

  //earth[0] = (1.1574e-5)*cos(48/180*pi)*cos(current[0][1])*cos(current[0][2])+(-1.1574e-5)*sin(48/180*pi)*(cos(current[0][0])*sin(current[0][1])*cos(current[0][2])+sin(current[0][0])*sin(current[0][2]));
  //earth[1] = (1.1574e-5)*cos(48/180*pi)*cos(current[0][1])*sin(current[0][2])+(-1.1574e-5)*sin(48/180*pi)*(cos(current[0][0])*sin(current[0][1])*sin(current[0][2])-sin(current[0][0])*cos(current[0][2]));
  //earth[2] = (-1.1574e-5)*cos(48/180*pi)*sin(current[0][1])+(-1.1574e-5)*sin(48/180*pi)*cos(current[0][0])*cos(current[0][1]);

  //data[0][4] = data[0][4]-earth[0];
  //data[0][5] = data[0][5]-earth[1];
  //data[0][6] = data[0][6]-earth[2];

  //change[0][0] = data[0][4]+tan(current[0][1])*(data[0][5]*sin(current[0][0])+data[0][6]*cos(current[0][0]));
  //change[0][1] = data[0][5]*cos(current[0][0])-data[0][6]*sin(current[0][0]);
  //change[0][2] = (1/cos(current[0][1]))*(data[0][5]*sin(current[0][0])+data[0][6]*cos(current[0][0]));
  change[1][0] = data[0][1]-data[0][5]*current[1][2]+data[0][6]*current[1][1]+g*s01;
  change[1][1] = data[0][2]-data[0][6]*current[1][0]+data[0][4]*current[1][2]-g*c01*s00;
  change[1][2] = data[0][3]-data[0][4]*current[1][1]+data[0][5]*current[1][0]-g*c01*c00;



  change[2][0] = current[1][0]*c01*cos(current[0][2])+current[1][1]*(s00*s01*cos(current[0][2])-c00*sin(current[0][2]))+current[1][2]*(c00*s01*cos(current[0][2])+s00*sin(current[0][2]));
  change[2][1] = current[1][0]*c01*sin(current[0][2])+current[1][1]*(s00*s01*sin(current[0][2])+c00*sin(current[0][2]))+current[1][2]*(c00*s01*sin(current[0][2])-s00*cos(current[0][2]));
  change[2][2] = -current[1][0]*s01+current[1][1]*s00*c01+current[1][2]*c00*c01;

  //current[0][0] = current[0][0] + (change[0][0]*dt);
  //current[0][1] = current[0][1] + (change[0][1]*dt);
  //current[0][2] = current[0][2] + (change[0][2]*dt);
  current[1][0] = current[1][0] + (change[1][0]*dt);
  current[1][1] = current[1][1] + (change[1][1]*dt);
  current[1][2] = current[1][2] + (change[1][2]*dt);
  current[2][0] = current[2][0] + (change[2][0]*dt);
  current[2][1] = current[2][1] + (change[2][1]*dt);
  current[2][2] = current[2][2] + (change[2][2]*dt);

  current[0][0] = change_m[0][0];
  current[0][1] = change_m[0][1];
  current[0][2] = change_m[0][2];

  Qua_new[0] = Qua_m[0];
  Qua_new[1] = Qua_m[1];
  Qua_new[2] = Qua_m[2];
  Qua_new[3] = Qua_m[3];

  current[0][0] = fmod((current[0][0]*180/pi),360)/180*pi;
  current[0][1] = fmod((current[0][1]*180/pi),360)/180*pi;
  current[0][2] = fmod((current[0][2]*180/pi),360)/180*pi;

  if (current[0][0]<-pi){
      current[0][0]=current[0][0]+2*pi;}
  if (current[0][0]>pi){
      current[0][0]=current[0][0]-2*pi;}

  if (current[0][1]<-pi){
      current[0][1]=current[0][1]+2*pi;}
  if (current[0][1]>pi){
      current[0][1]=current[0][1]-2*pi;}
  if (current[0][1]<-pi/2){
      current[0][1]=-current[0][1]-pi;}
  if (current[0][1]>pi/2){
      current[0][1]=-current[0][1]+pi;}

  if (current[0][2]<-pi){
      current[0][2]=current[0][2]+2*pi;}
  if (current[0][2]>pi){
      current[0][2]=current[0][2]-2*pi;}

  output[0][0] = current[0][0];
  output[0][1] = current[0][1];
  output[0][2] = current[0][2];
  output[1][0] = current[1][0];
  output[1][1] = current[1][1];
  output[1][2] = current[1][2];
  output[2][0] = current[2][0];
  output[2][1] = current[2][1];
  output[2][2] = current[2][2];

}
