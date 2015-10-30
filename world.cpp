#include "world.h"
#include <time.h>
#include <fstream>

double drand()
{
	return static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
}

World::World()
{
	srand(time(NULL));
	createfloor();
	pos = Vector2D(0, 1);
	man.Initialize(0, 2.3);
	addMan(man);
	controller = new Controller(&man);
	solver.addRay(&controller->SwRay);
	solver.addRay(&controller->StRay);
}

void World::createfloor()
{
	Body* last;
	Vector2D position(-5,0);
	for (int i = 0; i < 200; i++)
	{
		// addBox(position.x +(i*1.5), position.y);
		last = addBox(position.x, position.y);
		double angle = (drand() * 3 - 1.5) * 1.2 * i / 200.0;
		last->orient = angle;
		last->position = position -
			last->transform * ((mPolygon*)last->shape[0])->pt[3];
		position = last->position + last->transform * ((mPolygon*)last->shape[0])->pt[2];
	}
}

Body* World::addBox(double x, double y)
{
	mPolygon poly;
	mShape* shape = &poly;
	poly.SetBox(.75,.075);
	Body* b = new Body(&shape, 1, x, y, 1, true);
	solver.addBody(b);
	b->filtergroup = 3;
	b->Friction = .9;
	return b;
}

void World::addMan(Man& man)
{
	Body* b; Joint* j;
	while (b = man.RegisterBodies())
		solver.addBody(b);
	while (j = man.RegisterJoints())
		solver.addJoint(j);
}

void World::drawBodies()
{
	solver.drawBodies(false);
	glColor3f(0.5, 0.9, 0.6);
	Vector2D point = man.GetPos();
	glBegin(GL_POINTS);
	glVertex2d(point.x, point.y);
	glVertex2d(controller->elbow.x, controller->elbow.y);
	glVertex2d(controller->endpoint.x, controller->endpoint.y);
	glEnd();
	
	mRay* a = &controller->SwRay,* b = &controller->StRay;
	point = a->origin + a->direction * a->result * a->length;
	glBegin(GL_LINE_STRIP);
	glVertex2d(a->origin.x, a->origin.y);
	glVertex2d(point.x, point.y);
	glEnd();
	point = b->origin + b->direction * b->result * b->length;
	glBegin(GL_LINE_STRIP);
	glVertex2d(b->origin.x, b->origin.y);
	glVertex2d(point.x, point.y);
	glEnd();
}

void World::Solve(double dt)
{
	// controller->Step(dt);
	// solver.Solve(dt);
	// controller->checkTransition(dt);
	for (int i = 0; i < 16; i++)
	{
		controller->Step(1/1000.0);
		solver.Solve(1/1000.0);
		controller->checkTransition(1/1000.0);
	}
}

Vector2D World::getPos()
{
	Vector2D ans = man.GetPos();
	Vector2D diff = pos-ans;
	pos.x -= diff.x * .05;
	pos.y -= diff.y * .05;
	return pos;
}

