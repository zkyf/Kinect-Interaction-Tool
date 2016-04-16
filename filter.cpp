#include "filter.h"

Filter::Filter() :
Kalman_ex(4, 4),
Kalman_ey(4, 4),
Kalman_vx(1, 1),
Kalman_vy(1, 1),
Kalman_C (1, 4),
Kalman_Sx(4, 1),
Kalman_Sy(4, 1),
Kalman_Gx(4, 1),
Kalman_Gy(4, 1)
{
	particleFilter.init();
}

bool cmpx(Joint &p1, Joint p2)
{
	return p1.Position.X >= p2.Position.X;
}

bool cmpy(Joint &p1, Joint p2)
{
	return p1.Position.Y >= p2.Position.Y;
}

void Filter::Median(Joint joint, double &dx, double &dy)
{
	Medians[Median_count++] = joint;
	Median_count = Median_count % Median_limit;
	Median_num++;
	int pos = 0;
	if (Median_num > Median_limit)
	{
		Median_num = Median_limit;
	}
	Joint temp[Median_limit + 1];
	for (int i = 0; i < Median_limit; i++)
	{
		temp[i] = Medians[i];
	}
	sort(&temp[0], &temp[Median_num], cmpx);
	dx = temp[Median_num / 2].Position.X;
	sort(&temp[0], &temp[Median_num], cmpy);
	dy = temp[Median_num / 2].Position.Y;
}

void Filter::Median_Clear()
{
	Median_count = 0;
	Median_num = 0;
	ZeroMemory(Medians, sizeof(Medians));
}

JOINTS Filter::Filter_Median_Gen(JOINTS joints)
{
	JOINTS Filter_Median;
	JOINTP i;
	double x, y;
	for (i = joints.begin();
			 i != joints.end();
			 i++)
	{
		Median(*i, x, y);
		Joint newJoint;
		newJoint.Position.X = x;
		newJoint.Position.Y = y;
		Filter_Median.push_back(newJoint);
	}
	return Filter_Median;
}

Joint Filter::Filter_Median(Joint now)
{
	double x, y;
	Median(now, x, y);
	Joint now_Median(now);
	now_Median.Position.X = x;
	now_Median.Position.Y = y;
	return now_Median;
}


void Filter::Average(Joint joint, double &dx, double &dy)
{
	int pos = Average_count;
	Averages[Average_count++] = joint;
	Average_count = Average_count % Average_limit;
	Average_num++;
	if (Average_num > Average_limit)
	{
		Average_num = Average_limit;
	}
	dx = 0;
	dy = 0;
	int sum = 0;
	for (int i = 0; i < Average_num; i++)
	{
		dx += Averages[pos].Position.X * (Average_num - i);
		dy += Averages[pos].Position.Y * (Average_num - i);
		pos--;
		if (pos < 0)
		{
			pos = Average_num - 1;
		}
		sum += Average_num - i;
	}
	dx /= sum;
	dy /= sum;
}

void Filter::Average_Clear()
{
	Average_count = 0;
	Average_num = 0;
	ZeroMemory(Averages, sizeof(Averages));
}

JOINTS Filter::Filter_Average_Gen(JOINTS joints)
{
	JOINTS Filter_Average;
	JOINTP i;
	double x, y;
	for (i = joints.begin();
			 i != joints.end();
			 i++)
	{
		Average(*i, x, y);
		Joint newJoint;
		newJoint.Position.X = x;
		newJoint.Position.Y = y;
		Filter_Average.push_back(newJoint);
	}
	return Filter_Average;
}

Joint Filter::Filter_Average(Joint now)
{
	double x, y;
	Average(now, x, y);
	Joint now_Average(now);
	now_Average.Position.X = x;
	now_Average.Position.Y = y;
	return now_Average;
}

void Filter::Polynal(Joint joint, double &dx, double &dy)
{
	double ex = 0, ey = 0;
	const double deltax = 0.1;
	const double deltay = 0.05;
	const double coe[4] = {
		-0.16666666666667, 1.0, -2.5, 2.666666666667
	};
	int pos = Polynal_count;
	for (int i = 0; i < Polynal_num; i++)
	{
		ex += coe[i] * Polynals[pos].Position.X;
		ey += coe[i] * Polynals[pos].Position.Y;
		pos++;
		pos = pos%Polynal_limit;
	}
	if (Polynal_num<Polynal_limit)
	{
		ex = joint.Position.X;
		ey = joint.Position.Y;
	}

	if (fabs(joint.Position.X - ex) > deltax ||
			fabs(joint.Position.Y - ey) > deltay)
	{
		joint.Position.X = ex;
		joint.Position.Y = ey;
	}

	dx = ex;
	dy = ey;

	Polynals[Polynal_count++] = joint;
	Polynal_count = Polynal_count % Polynal_limit;
	Polynal_num++;
	if (Polynal_num > Polynal_limit)
	{
		Polynal_num = Polynal_limit;
	}
}

void Filter::Polynal_Clear()
{
	ZeroMemory(Polynals, sizeof(Polynals));
	Polynal_count = 0;
	Polynal_num = 0;
}

Joint Filter::Filter_Polynal(Joint now)
{
	double x, y;
	Polynal(now, x, y);
	Joint now_Polynal(now);
	now_Polynal.Position.X = x;
	now_Polynal.Position.Y = y;
	return now_Polynal;
}

JOINTS Filter::Filter_Polynal_Gen(JOINTS joints)
{
	JOINTS Filter_Polynal;
	JOINTP i;
	double x, y;
	for (i = joints.begin();
			 i != joints.end();
			 i++)
	{
		Polynal(*i, x, y);
		Joint newJoint;
		newJoint.Position.X = x;
		newJoint.Position.Y = y;
		Filter_Polynal.push_back(newJoint);
	}
	return Filter_Polynal;
}

void Filter::Kalman(Joint joint, double &dx, double &dy)
{
	Kalmans[Kalman_count++] = joint;
	Kalman_count = Kalman_count % Kalman_limit;
	Kalman_num++;
	if (Kalman_num > Kalman_limit)
	{
		Kalman_num = Kalman_limit;
	}
	if (Kalman_num < Kalman_limit)
	{
		dx = joint.Position.X;
		dy = joint.Position.Y;
		return;
	}
	else
	{
		//X, Y
		int haha;
		haha = 1;
		double x[5], y[5];
		int pos = Kalman_count;
		for (int i = 0; i < 5; i++)
		{
			x[i] = Kalmans[pos].Position.X;
			y[i] = Kalmans[pos].Position.Y;
			pos++;
			pos = pos%Kalman_limit;
		}
		//求系数Ax, Ay
		double Ax[5] = {
			/*a0*/ x[0],
			/*a1*/ 4 * (x[1] - x[0]) - 3 * x[2] + 4 * x[3] / 3 - x[4] / 4,
			/*a2*/ 11 * x[4] / 24 - 7 * x[3] / 3 + 19 * x[2] / 4 - 13 * (x[1] - x[0]) / 3,
			/*a3*/ x[4] / 3 - 7 * x[3] / 6 + x[2] - (x[1] - x[0]) / 2,
			/*a4*/ (x[4] - 4 * x[3] + 6 * x[2] + 4 * (x[1] - x[0])) / 24
		};
		double Ay[5] = {
			/*a0*/ y[0],
			/*a1*/ 4 * (y[1] - y[0]) - 3 * y[2] + 4 * y[3] / 3 - y[4] / 4,
			/*a2*/ 11 * y[4] / 24 - 7 * y[3] / 3 + 19 * y[2] / 4 - 13 * (y[1] - y[0]) / 3,
			/*a3*/ y[4] / 3 - 7 * y[3] / 6 + y[2] - (y[1] - y[0]) / 2,
			/*a4*/ (y[4] - 4 * y[3] + 6 * y[2] + 4 * (y[1] - y[0])) / 24
		};

		//求转换矩阵Fx, Fy
		Matrix Fx(4, 4,
							 new double[16]{
			  1, 1, -0.5, (Ax[1] + 6 * Ax[3] - 4 * Ax[4]) / (24 * Ax[4]),
				0, 1, 1, 0.5,
				0, 0, 1, 1,
				0, 0, 0, 1});
		Matrix Fy(4, 4,
							 new double[16]{
			1, 1, -0.5, (Ay[1] + 6 * Ay[3] - 4 * Ay[4]) / (24 * Ay[4]),
				0, 1, 1, 0.5,
				0, 0, 1, 1,
				0, 0, 0, 1});
		//求ε(t|t-1)
		Matrix ex(4, 4), ey(4, 4);
		ex = Fx*Kalman_ex*(!Fx);
		ey = Fy*Kalman_ey*(!Fy);
		//cout << "ex" << endl; ex.print();
		//cout << "ey" << endl; ey.print();

		Matrix Bx(4, 1), By(4, 1);
		//cout << "!Kalman_C" << endl; (!Kalman_C).print();
		//cout << "Kalman_vx" << endl; Kalman_vx.print();
		//cout << "Kalman_C" << endl; Kalman_C.print();
		//cout << "!Kalman_C" << endl; (!Kalman_C).print();
		//cout << "Kalman_C*ex" << endl; (Kalman_C*ex).print();
		//cout << "Kalman_C*ex*(!Kalman_C)" << endl; (Kalman_C*ex*(!Kalman_C)).print();
		//cout << "(~(Kalman_vx + Kalman_C*ex*(!Kalman_C)))" << endl;
		//(~(Kalman_vx + Kalman_C*ex*(!Kalman_C))).print();
		Bx = ex*(!Kalman_C)*(~(Kalman_vx + Kalman_C*ex*(!Kalman_C)));
		//cout << "Bx" << endl; Bx.print();
		By = ey*(!Kalman_C)*(~(Kalman_vy + Kalman_C*ey*(!Kalman_C)));
		
		Matrix I4(4, 4);
		I4.SetIdentity();
		Kalman_Sx = (I4 - Bx*Kalman_C)*(Fx*Kalman_Sx + Kalman_Gx) +
			Bx*Matrix(1, 1, new double[1] {joint.Position.X});
		//cout << "Kalman_Sx" << endl; Kalman_Sx.print();
		Kalman_Sy = (I4 - By*Kalman_C)*(Fy*Kalman_Sy + Kalman_Gy) +
			By*Matrix(1, 1, new double[1] {joint.Position.Y});

		Kalman_ex = ex - Bx*Kalman_C*ex;
		Kalman_ey = ey - By*Kalman_C*ey;

		dx = Kalman_Sx.at(0, 0);
		dy = Kalman_Sy.at(0, 0);
	}
}

void Filter::Kalman_Set(Matrix C, Matrix vx, Matrix vy, Matrix ex, Matrix ey)
{
	Kalman_C = C;
	Kalman_vx = vx;
	Kalman_vy = vy;
	Kalman_ex = ex;
	Kalman_ey = ey;
}

Joint Filter::Filter_Kalman(Joint now)
{
	double x, y;
	Kalman(now, x, y);
	Joint now_Kalman(now);
	now_Kalman.Position.X = x;
	now_Kalman.Position.Y = y;
	return now_Kalman;
}

Joint Filter::JoyStick(Joint joint)
{
	double x = joint.Position.X + 0.005;
	double y = joint.Position.Y*1.2;
	double r = sqrt(x*x + y*y);
	double dr = 0.0;
	const double r0 = 0.05;
	const double r1 = 0.15;
	const double r2 = 0.4;
	const double s0 = 0.01;
	const double s1 = 0.04;
	if (fabs(r) <= r0)
	{
		dr = 0;
	}
	else if (fabs(r) <= r1)
	{
		dr = (fabs(r) - r0)*s0;
	}
	else if (fabs(r) <= r2)
	{
		dr = (fabs(r) - r1)*s1 + (r1-r0)*s0;
	}
	else
	{
		dr = (r2 - r1)*s1 + (r1 - r0)*s0;
	}
	double dx = dr / r * x;
	double dy = dr / r * y;

	Joint result;
	result = joint;
	result.Position.X = dx;
	result.Position.Y = dy;
	return result;
}

void Filter::LeastSquareInit(int n, int m)
{
	LS_List.clear();
	LS_n = n;
	LS_m = m;
	LS_count = 0;
	if (n <= 0)
	{
		MessageBoxA(NULL, "non-positive number used to initialize LeastSquare FIlter", "Error", MB_OK);
		return;
	}
	LS_A = Mat(m, m, CV_64FC1, Scalar(0,0,0));
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < m; j++)
		{
			LS_A.at<double>(i, j) = 0;
			if (i == 0 && j == j)
			{
				LS_A.at<double>(i, j) = n;
				continue;
			}
			for (int k = 0; k < n; k++)
			{
				double t = LS_t0 + LS_delta*k;
				LS_A.at<double>(i, j) += pow(t, i + j);
			}
		}
	}
}

Joint Filter::Filter_LeastSquare(Joint joint)
{
	LS_count++;
	LS_List.push_back(joint);
	while (LS_count > LS_n)
	{
		LS_count--;
		LS_List.erase(LS_List.begin());
	}
	if (LS_count < LS_n)
	{
		return joint;
	}
	//Generate b
	Mat bx(LS_m, 1, CV_64FC1);
	Mat by(LS_m, 1, CV_64FC1);
	for (int i = 0; i < LS_m; i++)
	{
		bx.at<double>(i, 0) = 0;
		by.at<double>(i, 0) = 0;
		for (int j = 0; j < LS_n; j++)
		{
			double t = LS_t0 + j*LS_delta;

			double xt = LS_List[j].Position.X * pow(t, i);
			bx.at<double>(i, 0) += xt;

			double yt = LS_List[j].Position.Y * pow(t, i);
			by.at<double>(i, 0) += yt;
		}
	}
	Mat ax = LS_A.inv()*bx;
	Mat ay = LS_A.inv()*by;
	double x = 0; double y = 0;
	double t = LS_t0 + LS_n*LS_delta;
	for (int i = 0; i < LS_m; i++)
	{
		x += ax.at<double>(i, 0)*pow(t, i);
		y += ay.at<double>(i, 0)*pow(t, i);
	}
	Joint result;
	result = joint;
	result.Position.X = x;
	result.Position.Y = y;
	return result;
}


void Filter::Particle(Joint joint, double &dx, double &dy)
{
	Joint p = particleFilter.process(joint);
	dx = p.Position.X;
	dy = p.Position.Y;
}

//void Filter::Particle_Set(Matrix C, Matrix vx, Matrix vy, Matrix ex, Matrix ey)
//{
//}

Joint Filter::Filter_Particle(Joint now)
{
	double x, y;
	Particle(now, x, y);
	Joint now_Particle(now);
	now_Particle.Position.X = x;
	now_Particle.Position.Y = y;
	return now_Particle;
}

void Filter::Particle_Filter::init() {
	condens = cvCreateConDensation(stateNum,measureNum,sampleNum);
	lowerBound = cvCreateMat(stateNum, 1, CV_32F);
	upperBound = cvCreateMat(stateNum, 1, CV_32F);
	lowerBound = cvCreateMat(stateNum, 1, CV_32F);
	upperBound = cvCreateMat(stateNum, 1, CV_32F);
	cvmSet(lowerBound,0,0,0.0);
	cvmSet(upperBound,0,0,winWidth/8);
	cvmSet(lowerBound,1,0,0.0);
	cvmSet(upperBound,1,0,winHeight/8);
	cvmSet(lowerBound,2,0,0.0);
	cvmSet(upperBound,2,0,0.0);
	cvmSet(lowerBound,3,0,0.0);
	cvmSet(upperBound,3,0,0.0);
	float A[stateNum][stateNum] ={
		1,0,1,0,
		0,1,0,1,
		0,0,1,0,
		0,0,0,1
	};
	memcpy(condens->DynamMatr,A,sizeof(A));
	cvConDensInitSampleSet(condens, lowerBound, upperBound);

	CvRNG rng_state = cvRNG(0xffffffff);
	for(int i=0; i < sampleNum; i++){
		condens->flSamples[i][0] = float(cvRandInt( &rng_state ) % winWidth); //width
		condens->flSamples[i][1] = float(cvRandInt( &rng_state ) % winHeight);//height
	}
}

Joint Filter::Particle_Filter::process(Joint joint) {
	CvPoint mousePosition;
	mousePosition.x = (joint.Position.X+1)/2*winWidth;  //
	mousePosition.y = (joint.Position.Y+1)/2*winHeight;

	CvPoint predict_pt=cvPoint((int)condens->State[0],(int)condens->State[1]);

	float variance[measureNum]={0};		
	//get variance/standard deviation of each state
	for (int i=0;i<measureNum;i++) {
		//sum
		float sumState=0;
		for (int j=0;j<condens->SamplesNum;j++) {
			sumState+=condens->flSamples[i][j];
		}
		//average
		sumState/=sampleNum;
		//variance
		for (int j=0;j<condens->SamplesNum;j++) {
			variance[i]+=(condens->flSamples[i][j]-sumState)*
				(condens->flSamples[i][j]-sumState);
		}
		variance[i]/=sampleNum-1;
	}
	//3.update particals confidence
	CvPoint pt;
	if (isPredictOnly) {
		pt=predict_pt;
	}else{
		pt=mousePosition;
	}
	for (int i=0;i<condens->SamplesNum;i++) {
		float probX=(float)exp(-1*(pt.x-condens->flSamples[i][0])
			*(pt.x-condens->flSamples[i][0])/(2*variance[0]));
		float probY=(float)exp(-1*(pt.y-condens->flSamples[i][1])
			*(pt.y-condens->flSamples[i][1])/(2*variance[1]));
		condens->flConfidence[i]=probX*probY;
	}
	//4.update condensation
	cvConDensUpdateByTime(condens);
	
	
	Joint ret(joint);
	ret.Position.X = 1.0*predict_pt.x/winWidth*2-1;  //1.0
	ret.Position.Y = 1.0*predict_pt.y/winHeight*2-1;
	return ret;
}