#ifndef WORLD_H
#define WORLD_H

#include <algorithm>
#include "solver.h"
#include "man.h"

class World{
	public:
		World();
		
		void createfloor();
		Body* addBox(double x, double y);
		void addMan(Man& man);
		
		void drawBodies();
		void Solve(double dt);
		double GetProfile(std::string s) {return solver.GetProfile(s);}
		
		void SetSpeed(double speed) {controller->SetSpeed(speed);}
		Vector2D getPos();
	private:
		Vector2D pos;
		Solver solver;
		Man man;
		Controller* controller;
};

#endif