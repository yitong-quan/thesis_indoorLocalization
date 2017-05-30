#ifndef SOLVER_H
#define SOLVER_H

#include <math.h>
#include <vector>



#include "MatMath.h"
#include "Qrien_mag.h"
#include "Qrien_gyro.h"
#include "medianfilter.h"
#include "KF_IMU.h"



#define n_INI   500   //number of the data needed for initialization for foot
#define n_INI1  200   //number of the data needed for bias update
#define n_LP    20
#define med_L   30    //length of the median filter
#define N  1617      //length of the data
#define NN 150       //length of the temporay buffer for orien HDR
#define MM 100       //maximum time duration between 2 steps


void Solverini();
//void ComputeSolution(float* sensordata, int meas_num, float v, float p, float Orien);
void ComputeSolution(float* sensordata, int meas_num, float* v, float* p, float* orien_res, float* acc_res);

/*
class Solver : public QObject
{
	Q_OBJECT

	vector<SensorData*> * _MeasurementVector;

public:

	Solver(vector<SensorData*> * meas_vec);
	void ComputeSolution(int meas_num);

signals:

	void PositionComputed(Position * pos);
	void CTotalComputed(int value);
	void VelocityComputed(float x, float y, float z);
	void OrientationComputed(float roll, float pitch, float yaw);

};
*/

#endif

/*
#include <qobject.h>

#include "../Utility/BasicTypes.h"
#include "SolutionLib/MatMath.h"
#include "SolutionLib/Qrien_mag.h"
#include "SolutionLib/Qrien_gyro.h"
#include "SolutionLib/medianfilter.h"
#include "SolutionLib/KF_IMU.h"

#include "../Utility/BasicTypes.h"

#define n_INI   400   //number of the data needed for initialization for foot 
#define n_INI1   200   //number of the data needed for bias update 
#define n_LP    20
#define med_L   7    //length of the median filter 
#define N  1000      //length of the data
#define NN 150       //length of the temporay buffer for orien HDR
#define MM 100       //maximum time duration between 2 steps

class Solver : public QObject
{
Q_OBJECT

	vector<SensorData*> * _MeasurementVector;

public:

	Solver(vector<SensorData*> * meas_vec);
	void ComputeSolution(int meas_num);

signals:

	void PositionComputed(Position * pos);
	void CTotalComputed(int value);
	void VelocityComputed(float x, float y, float z);
	void OrientationComputed(float roll, float pitch, float yaw);

};

#endif
*/
