#include "collision.h"
#include <algorithm>
#include <iostream>

Manifold::Manifold(mShape *a, mShape *b) : A(a), B(b)
{
	normalImpulse[0] = 0;
	normalImpulse[1] = 0;
	tangentImpulse[0] = 0;
	tangentImpulse[1] = 0;
	e = std::max(A->body->restitution, B->body->restitution);
	df = std::sqrt(A->body->Friction * B->body->Friction);
}

void Manifold::Solve()
{
	Dispatch[A->GetType()][B->GetType()](this, A, B);
}

void Manifold::Initialize()
{
	double mA = A->body->planet ? 0 : A->body->mass.iM;
	double iA = A->body->planet ? 0 : A->body->mass.iI;
	double mB = B->body->planet ? 0 : B->body->mass.iM;
	double iB = B->body->planet ? 0 : B->body->mass.iI;
	Vector2D vA = A->body->velocity;
	Vector2D vB = B->body->velocity;
	double wA = A->body->angVel;
	double wB = B->body->angVel;
	for(int i = 0; i < contact_count; i++)
	{
		Vector2D rA = contacts[i] - A->body->position;
		Vector2D rB = contacts[i] - B->body->position;
		double rnA = cross(rA, normal);
		double rnB = cross(rB, normal);
		double kn = mA + mB + rnA * rnA * iA + rnB * rnB * iB;
		normalMass[i] = kn > 0 ? 1/kn : 0;
		double rtA = cross(rA, tangent);
		double rtB = cross(rB, tangent);
		double kt = mA + mB + rtA * rtA * iA + rtB * rtB * iB;
		tangentMass[i] = kt > 0 ? 1/kt : 0;
		if (true)
		{
		}
		else
		{
			normalImpulse[i] = 0;
			tangentImpulse[i] = 0;
		}
		velocityBias[i] = 0.0;
		Vector2D rv = vB + cross(wB, rB) - vA - cross(wA, rA);
		double vRel = dot(normal, rv);
		if (vRel < -1.0)
			velocityBias[i] = -e * vRel;
	}
	if (contact_count == 2)
	{
		Vector2D rA1 = contacts[0] - A->body->position;
		Vector2D rB1 = contacts[0] - B->body->position;
		Vector2D rA2 = contacts[1] - A->body->position;
		Vector2D rB2 = contacts[1] - B->body->position;
		double rn1A = cross(rA1, normal);
		double rn1B = cross(rB1, normal);
		double rn2A = cross(rA2, normal);
		double rn2B = cross(rB2, normal);
		double k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
		double k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
		double k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;
		const double k_maxConditionNumber = 1000.0;
		if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
		{
			K.Set(k11, k12, k12, k22);
			nMass = K.GetInverse();
		}
		else
			contact_count = 1;
	}
	if (contact_count == 1)
	{
		normalImpulse[1] = 0;
		tangentImpulse[1] = 0;
	}
}

void Manifold::WarmStart()
{
	double mA = A->body->planet ? 0 : A->body->mass.iM;
	double iA = A->body->planet ? 0 : A->body->mass.iI;
	double mB = B->body->planet ? 0 : B->body->mass.iM;
	double iB = B->body->planet ? 0 : B->body->mass.iI;
	for (int i = 0; i < contact_count; i++)
	{
		Vector2D rA = contacts[i] - A->body->position;
		Vector2D rB = contacts[i] - B->body->position;
		Vector2D P = normal * normalImpulse[i] + tangent * tangentImpulse[i];
		A->body->contactForce -= P;
		B->body->contactForce += P;
		A->body->velocity -= P * mA;
		A->body->angVel   -= iA * cross(rA, P);
		B->body->velocity += P * mB;
		B->body->angVel   += iB * cross(rB, P);
	}
}

void Manifold::ApplyImpulse()
{
	double mA = A->body->planet ? 0 : A->body->mass.iM;
	double iA = A->body->planet ? 0 : A->body->mass.iI;
	double mB = B->body->planet ? 0 : B->body->mass.iM;
	double iB = B->body->planet ? 0 : B->body->mass.iI;
	Vector2D vA = A->body->velocity;
	Vector2D vB = B->body->velocity;
	double wA = A->body->angVel;
	double wB = B->body->angVel;
	for (int i = 0; i < contact_count; i++)
	{
		Vector2D rA = contacts[i] - A->body->position;
		Vector2D rB = contacts[i] - B->body->position;
		Vector2D dv = vB + cross(wB, rB) - vA - cross(wA, rA);
		double vt = dot(dv, tangent);
		double lambda = tangentMass[i] * (-vt);
		double mF = df * normalImpulse[i];
		double newImpulse = Clamp(tangentImpulse[i] + lambda, -mF, mF);
		lambda = newImpulse - tangentImpulse[i];
		tangentImpulse[i] = newImpulse;
		Vector2D P = tangent * lambda;
		vA -= P * mA;
		wA -= iA * cross(rA, P);
		vB += P * mB;
		wB += iB * cross(rB, P);
	}
	if (contact_count == 1)
	{
		Vector2D rA = contacts[0] - A->body->position;
		Vector2D rB = contacts[0] - B->body->position;
		Vector2D dv = vB + cross(wB, rB) - vA - cross(wA, rA);
		double vn = dot(dv, normal);
		double lambda = -normalMass[0] * (vn - velocityBias[0]);
		double newImpulse = std::max(normalImpulse[0] + lambda, 0.0);
		lambda = newImpulse - normalImpulse[0];
		normalImpulse[0] = newImpulse;
		Vector2D P = normal * lambda;
		vA -= P * mA;
		wA -= iA * cross(rA, P);
		vB += P * mB;
		wB += iB * cross(rB, P);
	}
	else
	{
		Vector2D rA0 = contacts[0] - A->body->position;
		Vector2D rB0 = contacts[0] - B->body->position;
		Vector2D rA1 = contacts[1] - A->body->position;
		Vector2D rB1 = contacts[1] - B->body->position;
		Vector2D a(normalImpulse[0], normalImpulse[1]);
		Vector2D dv1 = vB + cross(wB, rB0) - vA - cross(wA, rA0);
		Vector2D dv2 = vB + cross(wB, rB1) - vA - cross(wA, rA1);
		double vn1 = dot(dv1, normal);
		double vn2 = dot(dv2, normal);
		Vector2D b;
		b.x = vn1 - velocityBias[0];
		b.y = vn2 - velocityBias[1];
		b -= K * a;
		for (;;)
		{
			Vector2D x = - (nMass * b);
			if (x.x >= 0.0 && x.y >= 0.0)
			{
				Vector2D d = x - a;
				Vector2D P1 = normal * d.x;
				Vector2D P2 = normal * d.y;
				vA -= (P1 + P2) * mA;
				wA -= iA * (cross(rA0, P1) + cross(rA1, P2));
				vB += (P1 + P2) * mB;
				wB += iB * (cross(rB0, P1) + cross(rB1, P2));
				normalImpulse[0] = x.x;
				normalImpulse[1] = x.y;
				break;
			}
			x.x = -normalMass[0] * b.x;
			x.y = 0.0;
			vn1 = 0.0;
			vn2 = K.m[1][0] * x.x + b.y;
			if (x.x >= 0.0 && vn2 >= 0.0)
			{
				Vector2D d = x - a;
				Vector2D P1 = normal * d.x;
				Vector2D P2 = normal * d.y;
				vA -= (P1 + P2) * mA;
				wA -= iA * (cross(rA0, P1) + cross(rA1, P2));
				vB += (P1 + P2) * mB;
				wB += iB * (cross(rB0, P1) + cross(rB1, P2));
				normalImpulse[0] = x.x;
				normalImpulse[1] = x.y;
				break;
			}
			x.x = 0.0;
			x.y = -normalMass[1] * b.y;
			vn1 = K.m[0][1] * x.y + b.x;
			vn2 = 0.0;
			if (x.y >= 0.0 && vn1 >= 0.0)
			{
				Vector2D d = x - a;
				Vector2D P1 = normal * d.x;
				Vector2D P2 = normal * d.y;
				vA -= (P1 + P2) * mA;
				wA -= iA * (cross(rA0, P1) + cross(rA1, P2));
				vB += (P1 + P2) * mB;
				wB += iB * (cross(rB0, P1) + cross(rB1, P2));
				normalImpulse[0] = x.x;
				normalImpulse[1] = x.y;
				break;
			}
			x.x = 0.0;
			x.y = 0.0;
			vn1 = b.x;
			vn2 = b.y;
			if (vn1 >= 0.0 && vn2 >= 0.0 )
			{
				Vector2D d = x - a;
				Vector2D P1 = normal * d.x;
				Vector2D P2 = normal * d.y;
				vA -= (P1 + P2) * mA;
				wA -= iA * (cross(rA0, P1) + cross(rA1, P2));
				vB += (P1 + P2) * mB;
				wB += iB * (cross(rB0, P1) + cross(rB1, P2));
				normalImpulse[0] = x.x;
				normalImpulse[1] = x.y;
				break;
			}
			break;
		}
	}
	A->body->contactForce -= (vA - A->body->velocity) * A->body->mass.m;
	B->body->contactForce += (vB - B->body->velocity) * B->body->mass.m;
	A->body->velocity = vA;
	A->body->angVel = wA;
	B->body->velocity = vB;
	B->body->angVel = wB;
}

void Manifold::PositionalCorrection()
{
	const double percent = 0.2;
	const double k_slop = .005;
	double mA = A->body->planet ? 0 : A->body->mass.iM;
	double iA = A->body->planet ? 0 : A->body->mass.iI;
	double mB = B->body->planet ? 0 : B->body->mass.iM;
	double iB = B->body->planet ? 0 : B->body->mass.iI;
	Vector2D cA = A->body->position;
	double aA = A->body->orient;
	Vector2D cB = B->body->position;
	double aB = B->body->orient;
	double R = 0;
	R += A->GetType() == mShape::_circle ? A->radius : 0;
	R += B->GetType() == mShape::_circle ? B->radius : 0;
	for (int i = 0; i < contact_count; i++)
	{
		Vector2D point;
		double separation;
		if (type == _circles)
		{
			Vector2D pointA = lpoint + A->body->position;
			Vector2D pointB = lpoints[0] + B->body->position;
			normal = pointB - pointA;
			normalize(normal);
			point = (pointA + pointB) * .5;
			separation = dot(pointB - pointA, normal) - R;
		}
		if (type == _faceA)
		{
			normal = A->body->transform * lnormal;
			Vector2D planepoint = A->body->transform * lpoint + A->body->position;
			Vector2D clippoint = B->body->transform * lpoints[i] + B->body->position;
			separation = dot(clippoint - planepoint, normal) - R;
			point = clippoint;
		}
		if (type == _faceB)
		{
			normal = B->body->transform * lnormal;
			Vector2D planepoint = B->body->transform * lpoint + B->body->position;
			Vector2D clippoint = A->body->transform * lpoints[i] + A->body->position;
			separation = dot(clippoint - planepoint, normal) - R;
			point = clippoint;
			normal = -normal;
		}
		Vector2D rA = point - A->body->position;
		Vector2D rB = point - B->body->position;
		double C = Clamp(percent * (separation + k_slop), -.2, 0);
		double rnA = cross(rA, normal);
		double rnB = cross(rB, normal);
		double K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
		double impulse = K > 0 ? - C / K : 0;
		Vector2D P = normal * impulse;
		cA -= P * mA;
		aA -= iA * cross(rA, P);
		cB += P * mB;
		aB += iB * cross(rB, P);
	}
	A->body->position = cA;
	A->body->orient = aA;
	B->body->position = cB;
	B->body->orient = aB;
}

CollisionCallback Dispatch[mShape::_count][mShape::_count] =
{
	{CircletoCircle, CircletoPolygon},
	{PolygontoCircle, PolygontoPolygon}
};

void CircletoCircle(Manifold* m, mShape* a, mShape* b)
{
	mCircle *A = reinterpret_cast<mCircle*>(a);
	mCircle *B = reinterpret_cast<mCircle*>(b);
	m->normal = b->body->position - a->body->position;
	double dist = m->normal.LSqr();
	double radius = A->radius + B->radius;
	if(dist > radius * radius)
	{
		m->contact_count = 0;
		return;
	}
	m->contact_count = 1;
	m->contacts[0] = (b->body->position + a->body->position) / 2;
	m->type = _circles;
	normalize(m->normal);
	m->tangent = cross(1, m->normal);
	m->lpoint = Vector2D(0,0);
	m->lnormal = Vector2D(0,0);
	m->lpoints[0] = Vector2D(0,0);
}

void CircletoPolygon(Manifold* m, mShape* a, mShape* b)
{
	mCircle* A  = reinterpret_cast<mCircle*>	(a);
	mPolygon* B = reinterpret_cast<mPolygon*>	(b);
	m->contact_count = 0;
	Vector2D center = a->body->position;
	center = B->body->transform.transpose() * (center - b->body->position);
	double separation = -FLT_MAX;
	int edgeNormal = 0;
	for(int i = 0; i < B->vertexCount; i++)
	{
		double s = dot(B->norm[i], center - B->pt[i]);
		if(s > A->radius)
			return;
		if(s > separation)
		{
			separation = s;
			edgeNormal = i;
		}
	}
	Vector2D v1 = B->pt[edgeNormal];
	int i2 = edgeNormal + 1 < B->vertexCount ? edgeNormal + 1 : 0;
	Vector2D v2 = B->pt[i2];
	if(separation < EPSILON)
	{
		m->contact_count = 1;
		m->type = _faceB;
		m->lnormal = B->norm[edgeNormal];
		m->lpoint = (v1 + v2) * .5;
		m->normal = -(B->body->transform * B->norm[edgeNormal]);
		m->lpoints[0].Set(0,0);
		m->contacts[0] = m->normal * A->radius + a->body->position;
		return;
	}
	double dot1 = dot(center - v1, v2 - v1);
	double dot2 = dot(center - v2, v1 - v2);
	if(dot1 <= 0)
	{
		if(DistSqr(center, v1) > A->radius * A->radius)
			return;
		m->contact_count = 1;
		m->type = _faceB;
		m->lnormal = center - v1;
		normalize(m->lnormal);
		m->normal = -(B->body->transform * m->lnormal);
		normalize(m->normal);
		m->lpoint = v1;
		m->lpoints[0].Set(0,0);
		v1 = B->body->transform * v1 + b->body->position;
		m->contacts[0] = v1;
	}
	else if(dot2 <= 0)
	{
		if(DistSqr(center, v2) > A->radius * A->radius)
			return;
		m->contact_count = 1;
		m->type = _faceB;
		m->lnormal = center - v2;
		normalize(m->lnormal);
		m->normal = -(B->body->transform * m->lnormal);
		normalize(m->normal);
		m->lpoint = v2;
		m->lpoints[0].Set(0,0);
		v2 = B->body->transform * v2 + b->body->position;
		m->contacts[0] = v2;
	}
	else
	{
		m->lpoint = (v1 + v2) * .5;
		m->lnormal = B->norm[edgeNormal];
		if(dot(center - m->lpoint, m->lnormal) > A->radius)
			return;
		m->normal = -(B->body->transform * m->lnormal);
		m->type = _faceB;
		m->lpoints[0].Set(0,0);
		m->contacts[0] = m->normal * A->radius + A->body->position;
		m->contact_count = 1;
	}
	m->tangent = cross(m->normal, 1);
}

void PolygontoCircle(Manifold* m, mShape* a, mShape* b)
{
	CircletoPolygon(m, b, a);
	m->type = _faceA;
	m->normal = -m->normal;
	m->tangent = -m->tangent;
}

double FindAxisMaxSeparation(int* edgeIndex, mPolygon* A, mPolygon* B )
{
	double bestDistance = -FLT_MAX;
	int bestIndex = 0;
	for(int i = 0; i < A->vertexCount; i++)
	{
		Vector2D n = A->norm[i];
		Vector2D nw = A->body->transform * n;
		Matrix2D btT = B->body->transform.transpose();
		n = btT * nw;
		Vector2D s = B->GetSupport(-n);
		Vector2D v = A->pt[i];
		v = A->body->transform * v + A->body->position;
		v -= B->body->position;
		v = btT * v;
		double d = dot(n, s - v);
		if(d > bestDistance)
		{
			bestDistance = d;
			bestIndex = i;
		}
	}
	*edgeIndex = bestIndex;
	return bestDistance;
}

void FindIncidentEdge(Vector2D* v, mPolygon* RefPoly, mPolygon* IncPoly, int referenceIndex)
{
	Vector2D referenceNormal = RefPoly->norm[referenceIndex];
	referenceNormal = RefPoly->body->transform * referenceNormal;
	referenceNormal = IncPoly->body->transform.transpose() * referenceNormal;
	int incidentEdge = 0;
	double minDot = FLT_MAX;
	for(int i = 0; i < IncPoly->vertexCount; i++)
	{
		double d = dot(referenceNormal, IncPoly->norm[i]);
		if(d < minDot)
		{
			minDot = d;
			incidentEdge = i;
		}
	}
	v[0] = IncPoly->body->transform * IncPoly->pt[incidentEdge] + IncPoly->body->position;
	incidentEdge = incidentEdge + 1 >= IncPoly->vertexCount ? 0 : incidentEdge + 1;
	v[1] = IncPoly->body->transform * IncPoly->pt[incidentEdge] + IncPoly->body->position;
}

int Clip(Vector2D* out, Vector2D* in, Vector2D n, double c)
{
	int num = 0;
	double dist0 = dot(n, in[0]) - c;
	double dist1 = dot(n, in[1]) - c;
	if (dist0 <= 0) out[num++] = in[0];
	if (dist1 <= 0) out[num++] = in[1];
	if (dist0 * dist1 < 0)
	{
		double intpnt = dist0 / (dist0 - dist1);
		out[num] = in[0] + (in[1] - in[0]) * intpnt;
		num++;
	}
	return num;
}

void PolygontoPolygon(Manifold* m, mShape* a, mShape* b)
{
	mPolygon *A = reinterpret_cast<mPolygon *>(a);
	mPolygon *B = reinterpret_cast<mPolygon *>(b);
	m->contact_count = 0;
	int edgeA = 0;
	double separationA = FindAxisMaxSeparation(&edgeA, A, B);
	if(separationA > 0)
		return;
	int edgeB = 0;
	double separationB = FindAxisMaxSeparation(&edgeB, B, A);
	if(separationB > 0)
		return;
	int referenceIndex;
	bool flip;
	mPolygon *RefPoly; // Reference
	mPolygon *IncPoly; // Incident
	if(separationB > separationA + .0005)
	{
		RefPoly = B;
		IncPoly = A;
		referenceIndex = edgeB;
		m->type = _faceB;
		flip = true;
	}
	else
	{
		RefPoly = A;
		IncPoly = B;
		referenceIndex = edgeA;
		m->type = _faceA;
		flip = false;
	}
	Vector2D incidentEdge[2];
	FindIncidentEdge(incidentEdge, RefPoly, IncPoly, referenceIndex);
	Vector2D v1 = RefPoly->pt[referenceIndex];
	referenceIndex = referenceIndex + 1 < RefPoly->vertexCount ? referenceIndex + 1 : 0;
	Vector2D v2 = RefPoly->pt[referenceIndex];
	m->ltangent = v2-v1;
	normalize(m->ltangent);
	m->lnormal = cross(m->ltangent, 1);
	m->lpoint = (v1+v2) * .5;
	Vector2D tangent = RefPoly->body->transform * m->ltangent;
	Vector2D normal = cross(tangent, 1);
	v1 = RefPoly->body->transform * v1 + RefPoly->body->position;
	v2 = RefPoly->body->transform * v2 + RefPoly->body->position;
	
	double refC = dot(normal, v1);
	double negSide = -dot(tangent, v1);
	double posSide =  dot(tangent, v2);
	
	Vector2D clip1[2];
	Vector2D clip2[2];
	int np;
	
	np = Clip(clip1, incidentEdge, -tangent, negSide);
	if (np < 2) return;
	np = Clip(clip2, clip1, tangent, posSide);
	if (np < 2) return;
	int cp = 0;
	
	for (int i = 0; i < 2; i++)
	{
		double separation = dot(normal, clip2[i]) - refC;
		if (separation <= 0)
		{
			m->contacts[cp] = clip2[i];
			m->lpoints[cp] = IncPoly->body->transform.transpose() * 
					(clip2[i] - IncPoly->body->position);
			cp++;
		}
	}
	m->contact_count = cp;
	m->normal = flip ? -normal : normal;
	m->tangent = flip ? -tangent : tangent;
}