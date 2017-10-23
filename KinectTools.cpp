#include "KinectTools.h"

JOINT::JOINT() {}
JOINT::JOINT(Joint p) : Joint(p) {}
double JOINT::length()
{
	double x = Position.X;
	double y = Position.Y;
	double z = Position.Z;
	return sqrt(x*x + y*y + z*z);
}

void JOINT::RotX(double radian)
{
	_3DPoint &p = Position;
	p.Y = p.Y*cos(radian) - p.Z*sin(radian);
	p.Z = p.Y*sin(radian) + p.Z*cos(radian);
}

void JOINT::RotY(double radian)
{
	_3DPoint &p = Position;
	p.X = p.X*cos(radian) - p.Z*sin(radian);
	p.Z = p.X*sin(radian) + p.Z*cos(radian);
}

void JOINT::RotZ(double radian)
{
	_3DPoint &p = Position;
	p.X = p.X*cos(radian) - p.Y*sin(radian);
	p.Y = p.X*sin(radian) + p.Y*cos(radian);
}

void JOINT::RotA(_3DPoint a, double radian)
{
	double r = sqrt(a.X*a.X + a.Y*a.Y + a.Z*a.Z);
	if (r <= 1e-10)
	{
		return;
	}
	a.X /= r; a.Y /= r; a.Z /= r;
	double x = a.X, y = a.Y, z = a.Z;
	double rx = asin(z / sqrt(y*y + z*z));
	if (y < 0)
	{
		rx = M_PI - rx;
	}
	RotX(rx);
	double rz = acos(x / sqrt(x*x + y*y + z*z));
	RotZ(rz);
	RotX(radian);
	RotZ(-rz);
	RotX(-rx);
}

JOINT operator-(JOINT a, JOINT b)
{
	JOINT result = a;
	result.Position.X -= b.Position.X;
	result.Position.Y -= b.Position.Y;
	result.Position.Z -= b.Position.Z;
	return result;
}

JOINT operator+(JOINT a, JOINT b)
{
	JOINT result = a;
	result.Position.X += b.Position.X;
	result.Position.Y += b.Position.Y;
	result.Position.Z += b.Position.Z;
	return result;
}

JOINT operator*(JOINT a, double b)
{
	JOINT result = a;
	result.Position.X *= b;
	result.Position.Y *= b;
	result.Position.Z *= b;
	return result;
}
JOINT operator*(double a, JOINT b) { return b*a; }

JOINT operator/(JOINT a, double b)
{
	Joint result = a;
	result.Position.X /= b;
	result.Position.Y /= b;
	result.Position.Z /= b;
	return result;
}

JOINTS::JOINTS() {}
JOINTS::JOINTS(JOINTS& ps)
{
	clear();
	for (int i = 0; i< ps.size(); i++)
	{
		push_back(ps[i]);
	}
}
void JOINTS::operator=(JOINTS& ps)
{
	clear();
	for (int i = 0; i < ps.size(); i++)
	{
		push_back(ps[i]);
	}
}

double JOINTS::length()
{
	double len = 0;
	for (int i = 1; i < size(); i++)
	{
		double x = at(i).Position.X - at(i - 1).Position.X;
		double y = at(i).Position.Y - at(i - 1).Position.Y;
		len += sqrt(x*x + y*y);
	}
	return len;
}

void JOINTS::RotX(double r)
{
	for (int i = 0; i < size(); i++)
	{
		at(i).RotX(r);
	}
}

void JOINTS::RotZ(double r)
{
	for (int i = 0; i < size(); i++)
	{
		at(i).RotZ(r);
	}
}

void JOINTS::RotA(_3DPoint a, double r)
{
	for (int i = 0; i < size(); i++)
	{
		at(i).RotA(a, r);
	}
}

void JOINTS::Resample(int T)
{
	double avestep = length() / (T - 1);
	double acculen = 0;
	double step = 0;
	JOINTS res;
	if (size() == 0) return;
	res.push_back(at(0));
	for (int i = 1; i < size(); i++)
	{
		JOINT diff = at(i) - at(i - 1);
		step = diff.length();
		if (acculen + step >= avestep - 1e-10)
		{
			JOINT newp = at(i) + diff * ((avestep - acculen) / step);
			res.push_back(newp);
			insert(begin() + i, newp);
		}
		else
		{
			acculen += step;
		}
	}
	*this = res;
}

void JOINTS::Normalize()
{
	JOINTS& Joints = *this;
	double xmin = 1e20;
	double xmax = -1e20;
	for (int i = 0; i < Joints.size(); i++)
	{
		if (Joints[i].Position.X>xmax)
		{
			xmax = Joints[i].Position.X;
		}
		if (Joints[i].Position.X < xmin)
		{
			xmin = Joints[i].Position.X;
		}
	}
	if (fabs(xmax - xmin)>KTEPS)
	{
		for (int i = 0; i < Joints.size(); i++)
		{
			Joints[i].Position.X = (Joints[i].Position.X - xmin) / (xmax - xmin);
		}
	}

	double ymin = 1e20;
	double ymax = -1e20;
	for (int i = 0; i < Joints.size(); i++)
	{
		if (Joints[i].Position.Y>ymax)
		{
			ymax = Joints[i].Position.Y;
		}
		if (Joints[i].Position.Y < ymin)
		{
			ymin = Joints[i].Position.Y;
		}
	}
	if (fabs(ymax - ymin)>KTEPS);
	{
		for (int i = 0; i < Joints.size(); i++)
		{
			Joints[i].Position.Y = (Joints[i].Position.Y - ymin) / (ymax - ymin);
		}
	}

	double zmin = 1e20;
	double zmax = -1e20;
	for (int i = 0; i < Joints.size(); i++)
	{
		if (Joints[i].Position.Z>zmax)
		{
			zmax = Joints[i].Position.Z;
		}
		if (Joints[i].Position.Z < zmin)
		{
			zmin = Joints[i].Position.Z;
		}
	}
	if (fabs(zmax - zmin)>KTEPS);
	{
		for (int i = 0; i < Joints.size(); i++)
		{
			Joints[i].Position.Z = (Joints[i].Position.Z - zmin) / (zmax - zmin);
		}
	}
}

void JOINTS::RotY(double r)
{
	for (int i = 0; i < size(); i++)
	{
		at(i).RotY(r);
	}
}

double JointsDistance(JOINTS p1, JOINTS p2)
{
	if (p1.size() != p2.size())
	{
		return -1;
	}
	double sum = 0;
	for (int i = 0; i < p1.size(); i++)
	{
		sum += (p1[i] - p2[i]).length();
	}
	return sum;
}