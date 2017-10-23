/*
 * KinectTools.h
 * Created by ZKYF
 * 
 * This header file add some basic math functions to the Joint class.
*/

#define _USE_MATH_DEFINES

#include <Windows.h>
#include <Kinect.h>
#include <math.h>
#include <vector>

using namespace std;

#ifndef _KINECTTOOLS_H
#define _KINECTTOOLS_H

const double KTEPS = 1e-10;

typedef CameraSpacePoint _3DPoint;

class JOINT : public Joint
{
	public:
  JOINT();
	JOINT(Joint p);
	double length();
	void RotX(double radian);
	void RotY(double radian);
	void RotZ(double radian);
	void RotA(_3DPoint a, double radian);

	friend JOINT operator+(JOINT a, JOINT b);
	friend JOINT operator-(JOINT a, JOINT b);
	friend JOINT operator*(JOINT a, double b);
	friend JOINT operator*(double a, JOINT b);
	friend JOINT operator/(JOINT a, double b);
};

typedef vector<JOINT> _JOINTS;

class JOINTS : public _JOINTS
{
	public:
	JOINTS();
	JOINTS(JOINTS& ps);
	void operator=(JOINTS& ps);
	double length();
	void RotX(double r);
	void RotY(double r);
	void RotZ(double r);
	void RotA(_3DPoint a, double r);
	void Resample(int T);
	void Normalize();
};

double JointsDistance(JOINTS p1, JOINTS p2);

typedef JOINTS::iterator JOINTP;
typedef vector<JOINTS> JOINTSLIST;
typedef vector<POINT> DRAWLIST;
typedef vector<POINT>::iterator DRAWITE;

#endif