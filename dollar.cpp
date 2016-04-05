/*
	还没完成的事：
		在窗口中加两个按钮，分别表示生成数据和检验手势。(是一直按下去的那种)
		如果数据只有一个点，为什么会返回-1？？
		score表示的是什么（为什么那个公式可以？）
		可以识别不同起始点的圆吗？做两次三分？
		xml手势数据的格式与读写
		算法优化（还是直接实现protractor来得快些）
*/


#include "dollar.h"

VEC::VEC() {}

VEC::VEC(double x,double y):x(x),y(y) {}

double VEC::len()
{
	return hypot(x,y);
}

double VEC::angle() 
{
	return atan2(y,x);
}

VEC VEC::rotateBy(double w)
{
		//和自己一样的，就是cos
	return VEC(x*cos(w)-y*sin(w),x*sin(w)+y*cos(w));
}

VEC operator+(VEC a,VEC b)
{
	return VEC(a.x+b.x,a.y+b.y);
}

VEC operator-(VEC a,VEC b) 
{
	return VEC(a.x-b.x,a.y-b.y);
}

VEC operator*(double t,VEC a)
{
	return VEC(a.x*t,a.y*t);
}

VEC operator/(VEC a,double b)
{
	return VEC(a.x/b,a.y/b);
}

VEC VEC::operator+=(VEC a)
{
	return *this=*this+a;
}

template<class T> void print(const vector<T> &a)
{
	for (int i = 0; i < a.size(); i++)
	{
		cout << a[i] << char(i == a.size() - 1 ? 10 : 32);
	}
}

template<class T> void println(const vector<T> &a)
{
	print(a);
	puts("");
}

double sqr(double a)
{
	return a*a;
}

istream &operator>>(istream &in,VEC &a)
{
	return in>>a.x>>a.y;
}

ostream &operator<<(ostream &out,const VEC &a) 
{
	return out<<a.x<<" "<<a.y;
}

double pathLength(Points p)
{
	double ret=0;
	for (int i=1;i<p.size();i++)
		ret+=(p[i]-p[i-1]).len();
	return ret;
}
Points resample(Points p,int n)
{
	if (p.size()<=0) { while (1) puts("a o"); exit(0); }
	if (p.size()==1) return vector<VEC>(n,p[0]);

	double I=pathLength(p)/(n-1);
	double D=0;
	Points q; q.push_back(p[0]);
	for (int i=1;i<p.size();i++) {
		double d=(p[i]-p[i-1]).len();
		if (D+d>=I-EPS) {  //EPS大法好\(^o^)/  (没有EPS的话，q数组的点数有可能就只有n-1个...)
			VEC newp=(I-D)/d*(p[i]-p[i-1])+p[i-1];  //VEC(p[i-1].x+(I-D)/d*(p[i].x-p[i-1].x), p[i-1].y+(I-D)/d*(p[i].y-p[i-1].y));
			q.push_back(newp);

			p.insert(p.begin()+i,newp);  //insert比较慢，需要优化  <--
			//当前的newp应该作为新的起点

			D=0;
		}
		else D+=d;
	}
	//cout<<(q.size()==n)<<" "<<I<<" "<<pathLength(q)/(n-1)<<endl;
	//cout<<*q.rbegin()<<" "<<*p.rbegin()<<endl;
	return q;
}

VEC centroid(Points p)
{
	VEC ret=VEC(0,0);
	for (int i=0;i<p.size();i++) ret+=p[i];
	return ret/p.size();
}

double indicativeAngle(Points p)
{
	VEC c=centroid(p);
	return (p[0]-c).angle();
}

Points rotateBy(Points p, double w)
{
	VEC c=centroid(p);
	Points newp;
	for (auto x:p)  newp.push_back((x-c).rotateBy(w)+c);  //计算角度时可以优化
	return newp;
}

VEC min(VEC a,VEC b)
{
	return VEC((a.x < b.x) ? a.x: b.x, (a.y < b.y) ? a.y: b.y);
}

VEC max(VEC a,VEC b)
{
	return VEC((a.x > b.x) ? a.x: b.x, (a.y > b.y) ? a.y: b.y);
}

VEC boundingBox(Points p)
{
	VEC ms=VEC(INF,INF),mx=VEC(-INF,-INF);
	for (auto x:p) ms=min(ms,x),mx=max(mx,x);
	return mx-ms;
}

Points scaleTo(Points p, double size)
{ 
	//把所有点给映射到一个正方形里
	VEC tmp=boundingBox(p); double width=tmp.x,height=tmp.y;
	Points newp;
	for (auto pp:p) newp.push_back(VEC(pp.x*size/width,pp.y*size/height));
	return newp;
}

Points translateTo(Points p,VEC k)
{ 
	//k=(0,0)
	VEC c=centroid(p);
	Points newp;
	for (auto pp:p) newp.push_back(pp+k-c);
	return newp;
}

double pathDistance(const Points &a,const Points &b)
{
	double d=0;
	for (int i=0;i<a.size();i++)
		d+=(b[i]-a[i]).len();
	return d/a.size();  //算的是平均值？	
}

double distanceAtAngle(Points p,Points T,double theta)
{
	Points newp=rotateBy(p,theta);
	return pathDistance(newp,T);
}

/*
 * 
 * 
*/
double distanecAtBestAngle(Points p,Points T,double Oa,double Ob,double delta)
{
	const double fi=(sqrt(5)-1)/2;

	double x1=fi*Oa+(1-fi)*Ob;
	double f1=distanceAtAngle(p,T,x1);
	double x2=(1-fi)*Oa+fi*Ob;
	double f2=distanceAtAngle(p,T,x2);
	while (fabs(Ob-Oa)>delta) {
		if (f1<f2) {
			Ob=x2;
			x2=x1;
			f2=f1;
			x1=fi*Oa+(1-fi)*Ob;
			f1=distanceAtAngle(p,T,x1);
		}
		else {
			Oa=x1;
			x1=x2;
			f1=f2;
			x2=(1-fi)*Oa+fi*Ob;
			f2=distanceAtAngle(p,T,x2);
		}
	}
	return (f1 < f2) ? f1 : f2;
}
Choice recognize(Points points, vector<Points> templates)
{  
	//template需要用其他类型来表示吗？不需要吧
	const double theta=PI/4,delta=PI*2.0/180;
	const double size=250;

	double b=INF;
	int id=-1;
	for (int i=0;i<templates.size();i++) {
		Points &t=templates[i];
		double d=distanecAtBestAngle(points,t,-theta,theta,delta);
		if (d<b) b=d,id=i;
	}

	double score=b/(0.5*sqrt(2*sqr(size)));  //??
	return make_pair(id,score);
}

Points normalize(Points a) 
{ 
	//归一化函数(所有手势数据都应该经过这个函数的处理)
	const VEC k=VEC(0,0);
	const double size=250;

	return scaleTo(translateTo(resample(a,32),k),size);
}

void pushpoint(int x, int y)
{
	points.push_back(VEC(x, y));
}

Choice check()
{
	points = normalize(points);
	Choice result = recognize(points, templates);
	return result;
}

void clear()
{
	points.clear();
}

/*
void on_mouse(int event, int x, int y, int flags, void* ustc)
{  
	//这个函数应该是新建线程去执行的...
    //CvFont font;  
    //cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);

	//连击的时候反应好慢...
	static bool mouseState = 0;
    if (event == CV_EVENT_LBUTTONDOWN) {
		if (!mouseState) {
			points.clear();
		}
		mouseState = 1;
    }
	if (event == CV_EVENT_LBUTTONUP) {
		if (mouseState) {
			points=normalize(points);
			//cout<<points.size()<<endl; println(points);
			pair<int,double> a=recognize(points,templates);
			cout<<a.first<<" "<<a.second<<endl;
			if (a.second<=0.4) puts("Yes! \\(^o^)/~");  //一般匹配了的手势，得到的score都是比0.4小的
			else puts("No. ╮(╯-╰)╭ ");
			puts("");
		}
		mouseState = 0;
	}
	if (event == CV_EVENT_MBUTTONDOWN) exit(0);
	if (mouseState) {
		points.push_back(VEC(x,y));
		circle(image,
			Point(x,y),
			8,
			Scalar( 0, 0, 255 ),
			-1
		);
		imshow("mouse",image);
	}
}

int main() {	
	{
		ifstream in("in");  //

		vector<VEC> v; VEC t;
		while (in>>t) v.push_back(t);
		v=normalize(v);
		cout<<v.size()<<endl; println(v);
		templates=vector<Points>(1,v);
	}

    cvNamedWindow("mouse");
    cvSetMouseCallback("mouse", on_mouse, 0);
	imshow("mouse",image);
	
	cvWaitKey(600000);
    cvDestroyAllWindows();

    return 0;
}
*/

/*
	Note:
	1.	ata(x)表示求的是x的反正切，其返回值为[-pi/2,+pi/2]之间的一个数。
		atan2(y,x)求的是y/x的反正切，其返回值为[-pi,+pi]之间的一个数。
*/

/*
	template怎么存？

	这个算法肯定能返回一个匹配的template？这是不对的，还应该设定一个距离和的上限值

	opencv鼠标事件反应好慢啊...
*/

/*
	参考：
	Mat结构：http://blog.csdn.net/yang_xian521/article/details/7107786
*/