#include "body.h"

int _shapes = 0;

Body::Body(mShape** s, int num, double x, double y, double density, bool p)
{
	for (int i = 0; i < num; i++)
	{
		shape.emplace_back(s[i]->Clone());
		shape[i]->body = this;
		shape[i]->aabb.shape = shape[i];
		shape[i]->id = _shapes++;
	}
	position.Set(x, y);
	velocity.Set(0, 0);
	contactForce.Set(0, 0);
	angVel = 0;
	torque = 0;
	orient = 0;
	force.Set(0, 0);
	Friction = 0.6;
	restitution = 0.2;
	filtergroup = 0;
	planet = p;
	SetMass(density);
	nodraw = false;
	contact = false;
	transform.Set(orient);
	for (int i = 0; i < shape.size(); i++)
		shape[i]->setAABB();
}
Body::~Body()
{
	for (int i = 0; i < shape.size(); i++)
		delete shape[i];
	shape.clear();
}
void Body::SetMass(double density)
{
	mass.Zero();
	for (int i = 0; i < shape.size(); i++)
	{
		massData data = shape[i]->ComputeMass(density);
		mass.m += data.m;
		mass.CoM += data.CoM * data.m;
		mass.I += data.I;
	}
	mass.iM = mass.m ? 1.0 / mass.m : 0.0;
	mass.iI = mass.I ? 1.0 / mass.I : 0.0;
	mass.CoM *= mass.iM;
	for (int i = 0; i < shape.size(); i++)
	{
		if (shape[i]->GetType() == mShape::_circle)
			continue;
		for (int j = 0; j < ((mPolygon*)shape[i])->vertexCount; j++)
			((mPolygon*)shape[i])->pt[j] -= mass.CoM;
	}
	position += mass.CoM;
}
void Body::applyG(double m, Vector2D dir, double dist)
{
	double g = G * m * mass.m / dist;
	Vector2D F = dir * g;
	ApplyForce(F);
}
void Body::ApplyImpulse(Vector2D impulse, Vector2D contactVector)
{
	velocity += impulse * mass.iM;
	angVel += mass.iI * cross(contactVector, impulse);
}
void Body::SetStatic()
{
	mass.I = 0.0f;
	mass.iI = 0.0f;
	mass.m = 0.0f;
	mass.iM = 0.0f;
}
bool AABBtest(AABB* a, AABB* b)
{
	if(a->max.x < b->min.x or a->min.x > b->max.x) return false;
	if(a->max.y < b->min.y or a->min.y > b->max.y) return false;
	return true;
}
void mShape::DrawAABB()
{
	Vector2D v = aabb.max;
	glVertex2d(v.x, v.y);
	v.x = aabb.min.x;
	glVertex2d(v.x, v.y);
	v.y = aabb.min.y;
	glVertex2d(v.x, v.y);
	v.x = aabb.max.x;
	glVertex2d(v.x, v.y);
	v = aabb.max;
	glVertex2d(v.x, v.y);
}
massData mCircle::ComputeMass(double density)
{
	massData mass;
	mass.m = pi * radius * radius * density;
	mass.I = mass.m * radius * radius;
	mass.CoM = Vector2D(0, 0);
	return mass;
}
void mCircle::Draw()
{
	int k_segments = 20;
	double theta = body->orient;
	double inc = pi * 2 / k_segments;
	for(int i = 0; i < k_segments; i++)
	{
		theta += inc;
		Vector2D p(std::cos(theta), std::sin(theta));
		p *= radius;
		p += body->position;
		glVertex2d(p.x, p.y);
	}
	glEnd();
	glBegin(GL_LINE_STRIP);
	glColor3f(1, 1, 1);
	Vector2D r(std::cos(body->orient), std::sin(body->orient));
	r *= radius;
	r += body->position;
	glVertex2d(body->position.x, body->position.y);
	glVertex2d(r.x, r.y);
}
void mCircle::setAABB()
{
	aabb.max = Vector2D(radius + .05, radius + .05) + body->position;
	aabb.min = Vector2D(-radius - .05, -radius -.05) + body->position;
}
bool mCircle::raycast(double& ans, Vector2D& norm, mRay ray)
{
	Vector2D s = ray.origin - body->position;
	double b = dot(s, s) - radius * radius;
	Vector2D r = ray.direction * ray.length;
	double c = dot(s, r);
	double rr = dot(r, r);
	double sigma = c * c - rr * b;
	if (sigma < 0 or rr < EPSILON)
		return false;
	float a = -(c + sqrt(sigma));
	if (a >= 0 and a <= rr)
	{
		ans = a / rr;
		norm = s + r * a;
		normalize(norm);
		return true;
	}
	return false;
}
mShape* mPolygon::Clone() const
{
	mPolygon* poly = new mPolygon();
	for(int i = 0; i < vertexCount; i++)
	{
		poly->pt[i] = pt[i];
		poly->norm[i] = norm[i];
	}
	poly->vertexCount = vertexCount;
	return poly;
}
massData mPolygon::ComputeMass(double density)
{
	radius = 0;
	massData mass;
	double area = 0;
	double I = 0;
	const double k_inv3 = 1.0 / 3.0;
	for(int i1 = 0; i1 < vertexCount; i1++)
	{
		Vector2D p1(pt[i1]);
		int i2 = i1 + 1 < vertexCount ? i1 + 1 : 0;
		Vector2D p2(pt[i2]);
		double D = cross(p1, p2);
		double triangleArea = 0.5 * D;
		area += triangleArea;
		mass.CoM += (p1 + p2) * triangleArea * k_inv3;
		double intx2 = p1.x * p1.x + p2.x * p1.x + p2.x * p2.x;
		double inty2 = p1.y * p1.y + p2.y * p1.y + p2.y * p2.y;
		I += (0.25 * k_inv3 * D) * (intx2 + inty2);
	}
	mass.CoM *= 1.0 / area;
	mass.m = density * area;
	mass.iM = mass.m ? 1.0 / mass.m : 0.0;
	mass.I = I * density;
	return mass;
}
void mPolygon::Draw()
{
	for(int i = 0; i < vertexCount; i++)
	{
		Vector2D v = body->position + body->transform * pt[i];
		glVertex2d(v.x, v.y);
	}
	Vector2D v = body->position + body->transform * pt[0];
	glVertex2d(v.x, v.y);
}
void mPolygon::setAABB()
{
	aabb.max.x = -FLT_MAX;
	aabb.max.y = -FLT_MAX;
	aabb.min.x = FLT_MAX;
	aabb.min.y = FLT_MAX;
	for(int i = 0; i < vertexCount; i++)
	{
		Vector2D v = body->position + body->transform * pt[i];
		if (v.x < aabb.min.x)	aabb.min.x = v.x;
		if (v.y < aabb.min.y)	aabb.min.y = v.y;
		if (v.x > aabb.max.x)	aabb.max.x = v.x;
		if (v.y > aabb.max.y)	aabb.max.y = v.y;
	}
	aabb.max.x += .05;
	aabb.max.y += .05;
	aabb.min.x -= .05;
	aabb.min.y -= .05;
}
bool mPolygon::raycast(double& ans, Vector2D& norm, mRay ray)
{
	Vector2D p1 = body->transform.transpose() * (ray.origin - body->position);
	Vector2D p2 = body->transform.transpose() * 
		(ray.origin + ray.direction * ray.length - body->position);
	Vector2D d = p2 - p1;
	
	double low = 0, hi = ray.length;
	int index = -1;
	for (int i = 0; i < vertexCount; i++)
	{
		double num = dot(this->norm[i], pt[i] -p1);
		double den = dot(this->norm[i], d);
		if (den == 0)
		{
			if (num < 0)
				return false;
		}
		else
		{
			if (den < 0 and num < low * den)
			{
				low = num / den;
				index = i;
			}
			else if (den > 0 and num < hi * den)
				hi = num / den;
		}
	}
	if (hi < low)
		return false;
	if (index >= 0)
	{
		ans = low;
		norm = body->transform * this->norm[index];
		return true;
	}
	return false;
}
void mPolygon::SetBox(double hw, double hh)
{
	vertexCount = 4;
	pt[0].Set(-hw, -hh);
	pt[1].Set( hw, -hh);
	pt[2].Set( hw,  hh);
	pt[3].Set(-hw,  hh);
	norm[0].Set( 0, -1);
	norm[1].Set( 1,  0);
	norm[2].Set( 0,  1);
	norm[3].Set(-1,  0);
}
void mPolygon::Set(Vector2D* vertices, int count)
{
	if (!(count > 2 && count <= MaxPolyVertexCount))
		return;
	int rightMost = 0;
	double maxX = vertices[0].x;
	for(int i = 1; i < count; i++)
	{
		double x = vertices[i].x;
		if(x > maxX)
		{
			maxX = x;
			rightMost = i;
		}
		else if(x == maxX)
			if(vertices[i].y < vertices[rightMost].y)
				rightMost = i;
	}
	int hull[MaxPolyVertexCount];
	int outCount = 0;
	int indexHull = rightMost;
	while(true)
	{
		hull[outCount] = indexHull;
		int nextHullIndex = 0;
		for(int i = 1; i < count; i++)
		{
			if(nextHullIndex == indexHull)
			{
				nextHullIndex = i;
				continue;
			}
			Vector2D e1 = vertices[nextHullIndex] - vertices[hull[outCount]];
			Vector2D e2 = vertices[i] - vertices[hull[outCount]];
			double c = cross(e1, e2);
			if(c < 0)
				nextHullIndex = i;
			if(c == 0 && e2.LSqr() > e1.LSqr())
				nextHullIndex = i;
		}
		outCount++;
		indexHull = nextHullIndex;
		if(nextHullIndex == rightMost)
		{
			vertexCount = outCount;
			break;
		}
	}
	for(int i = 0; i < vertexCount; i++)
		pt[i] = vertices[hull[i]];
	for(int i1 = 0; i1 < vertexCount; i1++)
	{
		int i2 = i1 + 1 < vertexCount ? i1 + 1 : 0;
		Vector2D edge = pt[i2] - pt[i1];
		norm[i1] = Vector2D( edge.y, -edge.x );
		normalize(norm[i1]);
	}
}
Vector2D mPolygon::GetSupport(const Vector2D& dir)
{
	double bestProjection = -FLT_MAX;
	Vector2D bestVertex;
	for(int i = 0; i < vertexCount; ++i)
	{
		Vector2D v = pt[i];
		double projection = dot(v, dir);
		if(projection > bestProjection)
		{
			bestVertex = v;
			bestProjection = projection;
		}
	}
	return bestVertex;
}
mShape* mRay::Clone() const
{
	return new mRay(origin, direction, length);
}
void mRay::setAABB()
{
	Vector2D o = origin;
	Vector2D e = origin + direction * length;
	aabb.max.x = (o.x > e.x ? o.x : e.x) + .05;
	aabb.max.y = (o.y > e.y ? o.y : e.y) + .05;
	aabb.min.x = (o.x < e.x ? o.x : e.x) - .05;
	aabb.min.y = (o.y < e.y ? o.y : e.y) - .05;
}
void mRay::prepareForBP()
{
	id = _shapes++;
	aabb.shape = this;
	setAABB();
}
bool mRay::cast()
{
	result = 2;
	double tmpf;
	Vector2D tmpn;
	for (int i = 0; i < shapes.size(); i++)
	{
		if (shapes[i]->raycast(tmpf, tmpn, *this) and tmpf < result)
		{
			result = tmpf;
			norm = tmpn;
		}
	}
	if (result <= 1)
		return true;
	return false;
}

