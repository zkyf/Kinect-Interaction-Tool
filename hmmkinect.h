#define _USE_MATH_DEFINES

#include <Windows.h>
#include <Kinect.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include "hmm.h"
#include "KinectTools.h"

using namespace std;

#ifndef _HMMKINECT_H
#define _HMMKINECT_H

const double SteadyThres = 0.03;

void clearZ(JOINTS& Joints)
{
	for (int i = 0; i < Joints.size(); i++)
	{
		Joints[i].Position.Z = 0;
	}
}

void SequenceGen(JOINTS& Joints, Sequence& seq)
{
	seq.clear();
	for (int i = 0; i < Joints.size() - 1; i++)
	{
		JOINT step = Joints[i + 1] - Joints[i];
		double angle = asin(step.Position.Y / step.length());
		if (step.Position.X < 0)
		{
			angle = M_PI - angle;
		}
		if (angle < 0)
		{
			angle += M_PI;
		}
		if (angle >= M_PI)
		{
			angle -= M_PI;
		}
		int state = angle / M_PI * 16;
		if (step.length() < SteadyThres)
		{
			state = 16;
		}
		seq.push_back(state);
	}
}

HMMResult matchAtAngle(JOINTS Joints, double radian, HMM& hmm)
{
	Joints.RotZ(radian);
	Sequence seq;
	SequenceGen(Joints, seq);
	return hmm.match(seq);
}

HMMResult match(JOINTS Joints, HMM& hmm, double thres = 0.1)
{
	const double k = (sqrt(5) - 1) / 2;
	double rmin = -M_PI / 2;
	double rmax = +M_PI / 2;
	double r1 = k*rmin + (1 - k)*rmax, r2 = k*rmax + (1 - k)*rmin;
	HMMResult f1 = matchAtAngle(Joints, r1, hmm);
	HMMResult f2 = matchAtAngle(Joints, r2, hmm);
	while (fabs(f1.partition - f2.partition) > thres)
	{
		if (f1 < f2)
		{
			rmax = r2;
			r2 = r1;
			f2 = f1;
			r1 = k*rmin + (1 - k)*rmax;
			f1 = matchAtAngle(Joints, r1, hmm);
		}
		else
		{
			rmin = r1;
			r1 = r2;
			f1 = f2;
			r2 = k*rmax + (1 - k)*rmin;
			f2 = matchAtAngle(Joints, r2, hmm);
		}
	}
	return (f1 > f2) ? f1 : f2;
}

#endif