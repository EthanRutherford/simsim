#include "base.h"

//Vector2D Begin
bool Vector2D::operator==(Vector2D P)
{
	return (Equal(x, P.x) and Equal(y, P.y));
}
bool Vector2D::operator!=(Vector2D P)
{
	return !(*this == P);
}
Vector2D Vector2D::operator-()
{
	return Vector2D(x != 0 ? -x: 0, y != 0 ? -y: 0);
}
Vector2D Vector2D::operator-(Vector2D P)
{
	return Vector2D(x - P.x, y - P.y);
}
void Vector2D::operator-=(Vector2D P)
{
	*this = *this - P;
}
Vector2D Vector2D::operator+(Vector2D P)
{
	return Vector2D(x + P.x, y + P.y);
}
void Vector2D::operator+=(Vector2D P)
{
	*this = *this + P;
}
Vector2D Vector2D::operator*(double S)
{
	return Vector2D(x * S, y * S);
}
void Vector2D::operator*=(double S)
{
	*this = *this * S;
}
Vector2D Vector2D::operator/(double S)
{
	return Vector2D(x / S, y / S);
}
void Vector2D::operator/=(double S)
{
	*this = *this / S;
}
Vector3D Vector3D::operator-()
{
	return Vector3D(-x, -y, -z);
}
Vector3D Vector3D::operator+(Vector3D P)
{
	return Vector3D(x + P.x, y + P.y, z + P.z);
}
void Vector3D::operator+=(Vector3D P)
{
	*this = *this + P;
}
//Vector2D End, Matrix2D Begin
Vector2D Matrix2D::operator*(Vector2D V)
{
	return Vector2D(m[0][0] * V.x + m[0][1] * V.y, m[1][0] * V.x + m[1][1] * V.y );
}
Vector3D Matrix3D::solve3(Vector3D b)
{
	Vector3D ex(m[0][0], m[1][0], m[2][0]);
	Vector3D ey(m[0][1], m[1][1], m[2][1]);
	Vector3D ez(m[0][2], m[1][2], m[2][2]);
	double det = dot(ex, cross(ey, ez));
	if (det != 0)
		det = 1 / det;
	Vector3D x;
	x.x = det * dot(b, cross(ey, ez));
	x.y = det * dot(ex, cross(b, ez));
	x.z = det * dot(ex, cross(ey, b));
	return x;
}
//Matrix2D End, trajectory1D Begin
void trajectory1D::addKnot(double t, double v)
{
	tValue.emplace_back(t);
	Value.emplace_back(v);
}
void trajectory1D::setKnotValue(int i, double v)
{
	if (i < Value.size())
		Value[i] = v;
}
void trajectory1D::setKnotPosition(int i, double p)
{
	if (i < tValue.size())
		tValue[i] = p;
}
void trajectory1D::clear()
{
	tValue.clear();
	Value.clear();
}
int trajectory1D::getFirstLargerIndex(double t)
{
	int size = tValue.size();
	if(size == 0) 
		return 0;
	for (int i = 0; i < size; i++)
		if (t < tValue[i])
			return i;
	return size;
}
double trajectory1D::evaluate_linear(double t)
{
	int size = tValue.size();
	if(size == 0) 				return 0;
	if (t <= tValue[0]) 		return Value[0];
	if (t >= tValue[size-1])	return Value[size-1];
	int index = getFirstLargerIndex(t);
	t = (t-tValue[index-1]) / (tValue[index]-tValue[index-1]);
	return (Value[index-1]) * (1-t) + (Value[index]) * t;
}
double trajectory1D::evaluate_catmull_rom(double t)
{
	int size = tValue.size();
	if(size == 0) return 0;
	if (t<=tValue[0]) return Value[0];
	if (t>=tValue[size-1])	return Value[size-1];
	int index = getFirstLargerIndex(t);
	t = (t-tValue[index-1]) / (tValue[index]-tValue[index-1]);
	double t0, t1, t2, t3;
	double p0, p1, p2, p3;
	p0 = (index-2 < 0) ? (Value[index-1]) : (Value[index-2]);
	p1 = Value[index-1];
	p2 = Value[index];
	p3 = (index+1 >= size) ? (Value[index]) : (Value[index+1]);
	t0 = (index-2 < 0) ? (tValue[index-1]) : (tValue[index-2]);
	t1 = tValue[index-1];
	t2 = tValue[index];
	t3 = (index+1 >= size) ? (tValue[index]) : (tValue[index+1]);
	double d1 = (t2-t0);
	double d2 = (t3-t1);
	if (d1 > -EPSILON && d1  < 0) d1 = -EPSILON;
	if (d1 < EPSILON && d1  >= 0) d1 = EPSILON;
	if (d2 > -EPSILON && d2  < 0) d2 = -EPSILON;
	if (d2 < EPSILON && d2  >= 0) d2 = EPSILON;
	double m1 = (p2 - p0) * (1-(t1-t0)/d1);
	double m2 = (p3 - p1) * (1-(t3-t2)/d2);
	t2 = t*t;
	t3 = t2*t;
	return p1*(2*t3-3*t2+1) + m1*(t3-2*t2+t) + p2*(-2*t3+3*t2) + m2 * (t3 - t2);
}
//trajectory1D End

void rotateAbout(Vector2D cpt, double angle, Vector2D& pt)
{
	double s = sin(angle);
	double c = cos(angle);
	pt -= cpt;
	double xnew = pt.x * c - pt.y * s;
	double ynew = pt.x * s + pt.y * c;
	pt.x = xnew + cpt.x;
	pt.y = ynew + cpt.y;
}

void rotateV(double angle, Vector2D& V)
{
	double s = sin(angle);
	double c = cos(angle);
	double xnew = V.x * c - V.y * s;
	double ynew = V.x * s + V.y * c;
	V.x = xnew;
	V.y = ynew;
}

double angleV(Vector2D V)
{
	Angle ans = atan2(V.y, V.x);
	return ans;
}

double angle(Vector2D a, Vector2D b)
{
	double x = b.x - a.x;
	double y = b.y - a.y;
	Angle ans = atan2(y, x);
	return ans;
}

Vector2D normal(Vector2D V)
{
	return Vector2D(-(V.y),V.x);
}

double dot(Vector2D a, Vector2D b)
{
	return (a.x * b.x) + (a.y * b.y);
}

double dot(Vector3D a, Vector3D b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

Matrix2D dot(Matrix2D a, Matrix2D b)
{
	return Matrix2D((a.m[0][0] * b.m[0][0]) + (a.m[0][1] * b.m[1][0]),
	(a.m[0][0] * b.m[0][1]) + (a.m[0][1] * b.m[1][1]),
	(a.m[1][0] * b.m[0][0]) + (a.m[1][1] * b.m[1][0]),
	(a.m[1][0] * b.m[0][1]) + (a.m[1][1] * b.m[1][1]));
}

double cross(Vector2D a, Vector2D b)
{
	return a.x * b.y - a.y * b.x;
}

Vector2D cross(Vector2D V, double a)
{
	return Vector2D(a * V.y, -a * V.x);
}

Vector2D cross(double a, Vector2D V)
{
	return Vector2D( -a * V.y, a * V.x );
}

Vector3D cross(Vector3D a, Vector3D b)
{
	return Vector3D(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

double mag(Vector2D V)
{
	return sqrt((V.x*V.x) + (V.y*V.y));
}

double Clamp(double num, double bottom, double top)
{
	if (num > top) return top;
	else if (num < bottom) return bottom;
	else return num;
}

void normalize(Vector2D& V)
{
	if (std::abs(V.x) + std::abs(V.y) > 0)
		V *= (1/mag(V));
}