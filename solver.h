#ifndef SOLVER_H
#define SOLVER_H

#include "profile.h"
#include "joint.h"
#include "collision.h"
#include "broadphase.h"
#include <vector>
#include <list>
#include <deque>

class Solver{
	public:
		Solver() : BPT(100), NPT(100), SLT(100), MVT(100) {applyg = NULL;}
		~Solver();
		void Solve(double dt);
		void addBody(Body* b);
		void addJoint(Joint* j);
		void addRay(mRay* r);
		
		void drawBodies(bool debug);
		double GetProfile(std::string s);
		void SetGravityFunc(void(* g)(std::deque<Body*>&)) {applyg = g;}
		
		void DeleteBody(Body* b);
		void Flush();
	private:
		void (* applyg)(std::deque<Body*>&);
		void applyG();
		void applyForces(double dt);
		void clearForces();
		void solvePositions(double dt);
		void checkCol();
		void solveVelocities(double dt);
		
		Profile BPT;
		Profile NPT;
		Profile SLT;
		Profile MVT;
		
		std::deque<Body*> body;
		std::vector<Joint*> joint;
		std::vector<mRay*> ray;
		std::list<Manifold> contacts;
		BroadPhase bp;
};

#endif