#include "Matrix.h"
#include "KinectTools.h"
#include <opencv2/opencv.hpp>
#include <math.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>


#ifndef _FILTER_H_
#define _FILTER_H_

using namespace ljxMat;
using namespace cv;

class Filter
{
	private:
	static const int Median_limit = 9;
	Joint Medians[Median_limit + 1];
	int Median_count = 0;
	int Median_num = 0;

	static const int Average_limit = 9;
	Joint Averages[Average_limit + 1];
	int Average_Coe[Average_limit];
	int Average_count = 0;
	int Average_num = 0;

	static const int Polynal_limit = 4;
	Joint Polynals[Polynal_limit + 1];
	int Polynal_count = 0;
	int Polynal_num = 0;

	static const int Kalman_limit = 5;
	Joint Kalmans[Kalman_limit + 1];
	int Kalman_count = 0;
	int Kalman_num = 0;
	Matrix Kalman_ex;/*(4, 4)*/
	Matrix Kalman_ey;/*(4, 4)*/
	Matrix Kalman_vx;/*(1, 1)*/
	Matrix Kalman_vy;/*(1, 1)*/
	Matrix Kalman_C ;/*(1, 4)*/
	Matrix Kalman_Sx;/*(4, 1)*/
	Matrix Kalman_Sy;/*(4, 1)*/
	Matrix Kalman_Gx;/*(4, 1)*/
	Matrix Kalman_Gy;/*(4, 1)*/
	
	struct Particle_Filter {
		static const int stateNum=4;
		static const int measureNum=2;
		static const int sampleNum=200;

		//CvPoint保存的是int整数，所以这里要大一些
		const int winHeight=10000;
		const int winWidth=10000;

		CvConDensation* condens;
		CvMat* lowerBound;
		CvMat* upperBound;
		bool isPredictOnly=false;
		void init();
		Joint process(Joint joint);
	} particleFilter;

	public:
	Filter();

	void Median(Joint joint, double &dx, double &dy);
	void Median_Clear();
	JOINTS Filter_Median_Gen(JOINTS joints);
	Joint Filter_Median(Joint now);

	void Average(Joint joint, double &dx, double &dy);
	void Average_Clear();
	Joint Filter_Average(Joint now);
	JOINTS Filter_Average_Gen(JOINTS joints);

	void Polynal(Joint joint, double &dx, double &dy);
	void Polynal_Clear();
	Joint Filter_Polynal(Joint now);
	JOINTS Filter_Polynal_Gen(JOINTS joints);

	void Kalman(Joint joint, double &dx, double &dy);
	void Kalman_Clear();
	void Kalman_Set(Matrix C, Matrix vx, Matrix vy, Matrix ex, Matrix ey);
	Joint Filter_Kalman(Joint now);
	
	Joint JoyStick(Joint joint);

	// Least Square Approximation
	const double LS_t0 = 0.65;
	const double LS_delta = 0.1;
	vector<Joint> LS_List;
	Mat LS_A;
	int LS_n;
	int LS_m;
	int LS_count;
	void LeastSquareInit(int n, int m = 4);
	Joint Filter_LeastSquare(Joint joint);

	
	void Particle(Joint joint, double &dx, double &dy);
	//void Particle_Clear();
	//void Particle_Set(Matrix C, Matrix vx, Matrix vy, Matrix ex, Matrix ey);
	Joint Filter_Particle(Joint now);
};


#endif