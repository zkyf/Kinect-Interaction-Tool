#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>

#ifndef _DOLLAR_H_
#define _DOLLAR_H_

using namespace std;

const double INF = 1e307;
const double PI = 3.141592653589793238462643383279;
const double EPS = 1e-10;
const int width = 600;
const double dollar_threshold = 0.4;

struct VEC
{
	double x, y;
	VEC();
	VEC(double x, double y);
	double len();
	double angle();
	VEC rotateBy(double w);
	friend VEC operator+(VEC a, VEC b);
	friend VEC operator-(VEC a, VEC b);
	friend VEC operator*(double t, VEC a);
	friend VEC operator/(VEC a, double b);
	VEC operator+=(VEC a);
};

typedef vector<VEC> Points;
typedef pair<int, double> Choice;

template<class T> void print(const vector<T> &a);
template<class T> void println(const vector<T> &a);

static vector<Points> templates;

static Points points;

double  sqr(double a);
istream &operator>>(istream &in, VEC &a);
ostream &operator<<(ostream &out, const VEC &a);
double  pathLength(Points p);
Points  resample(Points p, int n);
VEC     centroid(Points p);
double  indicativeAngle(Points p);
Points  rotateBy(Points p, double w);
VEC     boundingBox(Points p);
Points  scaleTo(Points p, double size);
Points  translateTo(Points p, VEC k);
double  pathDistance(const Points &a, const Points &b);
double  distanceAtAngle(Points p, Points T, double theta);
double  distanecAtBestAngle(Points p, Points T, double Oa, double Ob, double delta);
Choice  recognize(Points points, vector<Points> templates);
Points  normalize(Points a);
void    pushpoint(int x, int y);
Choice  check();
void    clear();
#endif