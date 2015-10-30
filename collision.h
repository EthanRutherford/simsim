#ifndef COLLISION_H
#define COLLISION_H

#include "body.h"

enum Type {
	_circles,
	_faceA,
	_faceB
};

struct Manifold{
	Manifold() {}
	Manifold(mShape *a, mShape *b);
	void Solve();					// Generate contact information
	void Initialize();				// Precalculations for impulse solving
	void WarmStart();				// Prepare for warm starting
	void ApplyImpulse();			// Solve impulse and apply
	void PositionalCorrection();	// Naive correction of positional penetration
	
	mShape* A;
	mShape* B;
	int contact_count;		// Number of contacts that occurred during collision
	double e;				// Mixed restitution
	double df;				// Mixed dynamic friction
	Type type;
	
	double penetration;
	Vector2D ltangent;
	Vector2D lnormal;
	Vector2D lpoint;
	Vector2D lpoints[2];
	
	Vector2D normal;
	Vector2D tangent;
	
	Vector2D contacts[2];	// Points of contact during collision
	double velocityBias[2];
	double normalImpulse[2];
	double tangentImpulse[2];
	double tangentMass[2];
	double normalMass[2];
	Matrix2D K;
	Matrix2D nMass;
};

typedef void (*CollisionCallback)(Manifold* m, mShape* a, mShape* b);
extern CollisionCallback Dispatch[mShape::_count][mShape::_count];
void CircletoCircle(Manifold* m, mShape* a, mShape* b);
void CircletoPolygon(Manifold* m, mShape* a, mShape* b);
void PolygontoCircle(Manifold* m, mShape* a, mShape* b);
void PolygontoPolygon(Manifold* m, mShape* a, mShape* b);

#endif