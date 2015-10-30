#ifndef BASE_H
#define BASE_H

#include <cmath>
#include <cfloat>
#include <vector>

static double pi = 3.14159265358979;
static double EPSILON = 0.0001;
static double G = 6;

struct Vector2D{
	double x;
	double y;
	bool operator==(Vector2D P);
	bool operator!=(Vector2D P);
	Vector2D operator-();
	Vector2D operator-(Vector2D P);
	void operator-=(Vector2D P);
	Vector2D operator+(Vector2D P);
	void operator+=(Vector2D P);
	Vector2D operator*(double S);
	void operator*=(double S);
	Vector2D operator/(double S);
	void operator/=(double S);
	Vector2D() : x(0), y(0) {}
	Vector2D(double a, double b) : x(a), y(b) {}
	void Set(double a, double b) {x = a; y = b;}
	double LSqr() {return x * x + y * y;}
};

struct Vector3D{
	double x, y, z;
	Vector3D operator-();
	Vector3D operator+(Vector3D P);
	void operator+=(Vector3D P);
	Vector3D() {}
	Vector3D(double a, double b, double c) : x(a), y(b), z(c) {}
	void Set(double a, double b, double c) {x = a; y = b; z = c;}
};

struct Matrix2D{
	double m[2][2];
	Matrix2D(double a, double b, double c, double d) : m{a, b, c, d} {}
	Matrix2D() : m{0,0,0,0} {}
	Vector2D operator*(Vector2D V);
	void Set(double a, double b, double c, double d){
		m[0][0] = a; m[0][1] = b; m[1][0] = c; m[1][1] = d;
	}
	void Set(double radians){
		double c = cos(radians);
		double s = sin(radians);
		m[0][0] = c; m[0][1] = -s;
		m[1][0] = s; m[1][1] =  c;
	}
	Matrix2D transpose() const {
		return Matrix2D(m[0][0], m[1][0], m[0][1], m[1][1]);
	}
	Matrix2D GetInverse() {
		double a = m[0][0], b = m[0][1], c = m[1][0], d = m[1][1];
		Matrix2D B;
		double det = a * d - b * c;
		if (det != 0.0)
			det = 1.0 / det;
		B.Set(det * d, -det * b, -det * c, det * a);
		return B;
	}
	Vector2D solve(Vector2D C) {
		double det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
		if (det != 0)
			det = 1 / det;
		return Vector2D(det * (m[1][1] * C.x - m[0][1] * C.y), 
			det * (m[0][0] * C.y - m[1][0] * C.x));
	}
};

struct RotMat2 : public Matrix2D{
	void Set(double& radians){
		rad = &radians;
		oldrad = radians+1;
		update();
	}
	Vector2D operator*(Vector2D V){
		update();
		return ((Matrix2D)*this)*(V);
	}
	Matrix2D transpose() {
		update();
		return Matrix2D::transpose();
	}
	private:
	void update() {
		if (oldrad != *rad)
		{
			Matrix2D::Set(*rad);
			oldrad = *rad;
		}
	}
	double* rad;
	double oldrad;
};

struct Matrix3D{
	double m[3][3];
	Matrix3D() {}
	Matrix3D(double a, double b, double c,double d, double e, double f, 
			double g, double h, double i) : m{a,b,c,d,e,f,g,h,i} {}
	Vector2D solve2(Vector2D C) {
		double det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
		if (det != 0)
			det = 1 / det;
		return Vector2D(det * (m[1][1] * C.x - m[0][1] * C.y), 
			det * (m[0][0] * C.y - m[1][0] * C.x));
	}
	Vector3D solve3(Vector3D b);
};

class Angle{
	public:
		Angle() {}
		Angle(double d) : a(d) {clean();}
		operator double&() {return a;}
		double& operator=(double d) {a = d; clean();}
		void operator+=(double d) {a += d; clean();}
		void operator-=(double d) {a -= d; clean();}
		void operator*=(double d) {a *= d; clean();}
		void operator/=(double d) {a /= d; clean();}
	private:
		void clean() {int s=a<0?-1:1; a -= 2*pi*(trunc((a+s*pi)/(2*pi)));}
		double a;
};

class trajectory1D{
	public:
		void addKnot(double t, double v);
		void setKnotValue(int i, double v);
		void setKnotPosition(int i, double p);
		int getKnotCount() {return tValue.size();}
		void clear();
		int getFirstLargerIndex(double t);
		double evaluate_linear(double t);
		double evaluate_catmull_rom(double t);
	private:
		std::vector<double> tValue;
		std::vector<double> Value;
};

void rotateAbout(Vector2D cpt, double angle, Vector2D& pt);
void rotateV(double angle, Vector2D& v);
double angleV(Vector2D V);
double angle(Vector2D a, Vector2D b);
Vector2D normal(Vector2D V);
double dot(Vector2D a, Vector2D b);
double dot(Vector3D a, Vector3D b);
Matrix2D dot(Matrix2D a, Matrix2D b);
double cross(Vector2D a, Vector2D b);
Vector2D cross(Vector2D V, double a);
Vector2D cross(double a, Vector2D V);
Vector3D cross(Vector3D a, Vector3D b);
double mag(Vector2D V);
void normalize(Vector2D& V);
double Clamp(double num, double bottom, double top);
inline bool Equal(double a, double b) {return std::abs(a - b) <= EPSILON;}
inline double sqr(double a) {return a*a;}
inline double DistSqr(Vector2D a, Vector2D b){return dot(a-b, a-b);}

#endif