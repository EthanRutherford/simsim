#include "man.h"

//Man

void Man::Initialize(double x, double y)
{
	Vector2D list1[] = {
		Vector2D(-.3, .6),
		Vector2D(.3, .6),
		Vector2D(.1, -.5),
		Vector2D(-.2, -.5)
	};
	Vector2D list2[] = {
		Vector2D(-.2, .5),
		Vector2D(.2, .5),
		Vector2D(.1, -.4),
		Vector2D(-.1, -.4)
	};
	Vector2D list3[] = {
		Vector2D(-.1, .4),
		Vector2D(.1, .4),
		Vector2D(0, -.4)
	};
	Vector2D list4[] = {
		Vector2D(-.2, .3),
		Vector2D(.2, .3),
		Vector2D(.1, -.2),
		Vector2D(-.2, -.2)
	};
	Vector2D list5[] = {
		Vector2D(-.1, .4),
		Vector2D(.2, .4),
		Vector2D(.2, -.4),
		Vector2D(-.1, -.4)
	};
	Vector2D list6[] = {
		Vector2D(-.1, .4),
		Vector2D(.2, .4),
		Vector2D(.1, -.4),
		Vector2D(-.1, -.4)
	};
	Vector2D list7[] = {
		Vector2D(-.1, .1),
		Vector2D(.4, 0),
		Vector2D(.4, -.1),
		Vector2D(-.1, -.1)
	};
	mPolygon poly1, poly2, poly3, poly4, poly5, poly6, poly7;
	poly1.Set(list1, 4);
	poly2.Set(list2, 4);
	poly3.Set(list3, 3);
	poly4.Set(list4, 4);
	poly5.Set(list5, 4);
	poly6.Set(list6, 4);
	poly7.Set(list7, 4);
	{ /* BODIES */
	mShape* shape = &poly3;	body.emplace_back(new Body(&shape, 1, x, y, .1, false));
	shape = &poly2;			body.emplace_back(new Body(&shape, 1, x, y, .1, false));
	shape = &poly7;			body.emplace_back(new Body(&shape, 1, x, y-2.0, .1, false));
	shape = &poly6;			body.emplace_back(new Body(&shape, 1, x, y-1.6, .1, false));
	shape = &poly5;			body.emplace_back(new Body(&shape, 1, x, y-.8, .1, false));
	shape = &poly1;			body.emplace_back(new Body(&shape, 1, x, y, .1, false));
	shape = &poly4;			body.emplace_back(new Body(&shape, 1, x, y+.6, .1, false));
	shape = &poly5;			body.emplace_back(new Body(&shape, 1, x, y-.8, .1, false));
	shape = &poly6;			body.emplace_back(new Body(&shape, 1, x, y-1.6, .1, false));
	shape = &poly7;			body.emplace_back(new Body(&shape, 1, x, y-2.0, .1, false));
	shape = &poly2;			body.emplace_back(new Body(&shape, 1, x, y, .1, false));
	shape = &poly3;			body.emplace_back(new Body(&shape, 1, x, y, .1, false));
	}
	{ /* JOINTS */
		joint.emplace_back(new RevJoint(body[5], body[1], Vector2D(0, .5), Vector2D(0, .45)));
		//((RevJoint*)joint.back())->SetLimit(true, pi, -(3*pi)/8);
		joint.emplace_back(new RevJoint(body[5], body[10], Vector2D(0, .5), Vector2D(0, .45)));
		//((RevJoint*)joint.back())->SetLimit(true, pi, -(3*pi)/8);
		joint.emplace_back(new RevJoint(body[1], body[0], Vector2D(0, -.45), Vector2D(0, .3)));
		((RevJoint*)joint.back())->SetLimit(true, (3*pi)/4, 0);
		joint.emplace_back(new RevJoint(body[10], body[11], Vector2D(0, -.45), Vector2D(0, .3)));
		((RevJoint*)joint.back())->SetLimit(true, (3*pi)/4, 0);
		joint.emplace_back(new RevJoint(body[5], body[6], Vector2D(0, .55), Vector2D(0, -.25)));
		((RevJoint*)joint.back())->SetLimit(true, pi/6, -pi/6);
		joint.emplace_back(new RevJoint(body[5], body[4], Vector2D(-.05, -.5), Vector2D(.05, .4)));
		((RevJoint*)joint.back())->SetLimit(true, (7*pi)/8, -pi/3);
		joint.emplace_back(new RevJoint(body[5], body[7], Vector2D(-.05, -.5), Vector2D(.05, .4)));
		((RevJoint*)joint.back())->SetLimit(true, (7*pi)/8, -pi/3);
		joint.emplace_back(new RevJoint(body[4], body[3], Vector2D(.05, -.4), Vector2D(.05, .4)));
		((RevJoint*)joint.back())->SetLimit(true, 0, -(7*pi)/8);
		joint.emplace_back(new RevJoint(body[7], body[8], Vector2D(.05, -.4), Vector2D(.05, .4)));
		((RevJoint*)joint.back())->SetLimit(true, 0, -(7*pi)/8);
		joint.emplace_back(new RevJoint(body[3], body[2], Vector2D(.05, -.4), Vector2D(.05, .05)));
		((RevJoint*)joint.back())->SetLimit(true, pi/4, -pi/2);
		joint.emplace_back(new RevJoint(body[8], body[9], Vector2D(.05, -.4), Vector2D(.05, .05)));
		((RevJoint*)joint.back())->SetLimit(true, pi/4, -pi/2);
	}
	for (int i = 0; i < body.size(); i++)
		body[i]->filtergroup = 1;
	b = j = 0;
	//make the feet grab the ground a little better
	body[back_foot]->Friction = .9;
	body[front_foot]->Friction = .9;
}

Body* Man::RegisterBodies()
{
	if (b == body.size())
		return NULL;
	return body[b++];
}

Joint* Man::RegisterJoints()
{
	if (j == joint.size())
		return NULL;
	return joint[j++];
}

Vector2D Man::GetPos()
{
	return comPosition;
}

void Man::updateCoM()
{
	Vector2D CoMPos(0, 0), CoMVel(0, 0);
	double totalmass = 0;
	for (Body* b : body)
	{
		totalmass += b->mass.m;
		CoMPos += b->position * b->mass.m;
		CoMVel += b->velocity * b->mass.m;
	}
	comPosition = CoMPos / totalmass;
	comVelocity = CoMVel / totalmass;
}

//Controller

Controller::Controller(Man* m) : man(m)
{
	SwRay.origin = Vector2D(0,.5);
	SwRay.direction.Set(0, -1);
	SwRay.length = 1;
	SwRay.filtergroup = m->body[torso]->filtergroup;
	StRay.origin = Vector2D(0,.5);
	StRay.direction.Set(0, -1);
	StRay.length = 1;
	StRay.filtergroup = m->body[torso]->filtergroup;
	
	stance = true;
	doubleStance = false;
	FSMStateIndex = 0;
	phi = 0;
	desiredvelocity = 0;
	setStanceSwing();
	updateCoM();
	SwFootStart = ((RevJoint*)SwAnkle)->GetAnchor();
	swingTraj.addKnot(0, 0);
	swingTraj.addKnot(1, 0);
	heightTraj.addKnot(0, .15);
	heightTraj.addKnot(.5, .26);
	heightTraj.addKnot(1, .16);
	
	for (int i = 0; i < man->joint.size(); i++)
	{
		desiredPose.joints.emplace_back();
		torques.emplace_back();
	}
	
	controlParams.emplace_back(man->joint[0], .3,  .1, 4 );		//back  shoulder
	controlParams.emplace_back(man->joint[1], .3,  .1, 4 );		//front shoulder
	controlParams.emplace_back(man->joint[2], .1, .02, 4 );		//back  elbow
	controlParams.emplace_back(man->joint[3], .1, .02, 4 );		//front elbow
	controlParams.emplace_back(man->joint[4], .2, .06, 4 );		//neck
	controlParams.emplace_back(man->joint[5],  6,  .6, 4 );		//back  hip
	controlParams.emplace_back(man->joint[6],  6,  .6, 4 );		//front hip
	controlParams.emplace_back(man->joint[7],  6,  .6, 20);		//back  knee
	controlParams.emplace_back(man->joint[8],  6,  .6, 20);		//front knee
	controlParams.emplace_back(man->joint[9],  1,  .3, 2 );		//back  ankle
	controlParams.emplace_back(man->joint[10], 1,  .3, 2 );		//front ankle
	rootControlParams.kp = 40;
	rootControlParams.kd = 8;
	rootControlParams.maxAbsTorque = 8;
	rootControlParams.strength = 1;
	
	loadFSM("simsim/walk.txt");
	
	//walking controller
	//states.emplace_back(0, .6, true, false, false, true);
	
	//states[0].traj.emplace_back(back_hip, front_hip); //stance hip
	
	// states[0].traj.emplace_back(front_hip, back_hip); //swing hip
	// states[0].traj[0].strength.addKnot(.2, .2);
	// states[0].traj[0].strength.addKnot(.4, 1);
	
	// states[0].traj.emplace_back(back_knee, front_knee); //stance knee
	// states[0].traj[1].comp.emplace_back();
	// states[0].traj[1].comp[0].base.addKnot(0.03344, -0.204846);
	// states[0].traj[1].comp[0].base.addKnot(0.959866, -0.070153);
	
	// states[0].traj.emplace_back(front_knee, back_knee); //swing knee
	// states[0].traj[2].strength.addKnot(.2, .2);
	// states[0].traj[2].strength.addKnot(.4, 1);
	// states[0].traj[2].comp.emplace_back();
	// states[0].traj[2].comp[0].base.addKnot(0, 0);
	
	// states[0].traj.emplace_back(back_ankle, front_ankle); //stance ankle
	// states[0].traj[3].strength.addKnot(.3, 1);
	// states[0].traj[3].comp.emplace_back();
	// states[0].traj[3].comp[0].base.addKnot(0, .1);
	// states[0].traj[3].comp[0].base.addKnot(.3, 0);
	// states[0].traj[3].comp[0].base.addKnot(.8, 0);
	// states[0].traj[3].comp[0].base.addKnot(1, -.2);
	// states[0].traj[3].comp[0].vscale.addKnot(-.1, .5);
	// states[0].traj[3].comp[0].vscale.addKnot(0, 0);
	// states[0].traj[3].comp[0].vscale.addKnot(.2, .2);
	// states[0].traj[3].comp[0].vscale.addKnot(.5, .2);
	// states[0].traj[3].comp[0].vscale.addKnot(1, 2.5);
	
	// states[0].traj.emplace_back(front_ankle, back_ankle); //swing ankle
	// states[0].traj[4].strength.addKnot(.2, .2);
	// states[0].traj[4].strength.addKnot(.4, 1);
	// states[0].traj[4].comp.emplace_back();
	// states[0].traj[4].comp[0].base.addKnot(0, -.3);
	// states[0].traj[4].comp[0].base.addKnot(.3, -.3);
	// states[0].traj[4].comp[0].base.addKnot(.4, 0);
	// states[0].traj[4].comp[0].base.addKnot(1, .1);
	// states[0].traj[4].comp[0].vscale.addKnot(-.5, 2);
	// states[0].traj[4].comp[0].vscale.addKnot(-.1, 1);
	// states[0].traj[4].comp[0].vscale.addKnot(0, 0);
	// states[0].traj[4].comp[0].vscale.addKnot(.1, 1);
	// states[0].traj[4].comp[0].vscale.addKnot(.5, 2.5);
	// states[0].traj[4].comp[0].vscale.addKnot(1, 6);
	// states[0].traj[4].comp[0].vscale.addKnot(1.1, 7);
	// states[0].traj[4].comp[0].vscale.addKnot(1.5, 3);
	
	// states[0].traj.emplace_back(back_shoulder, front_shoulder); //stance shoulder
	// states[0].traj[5].comp.emplace_back();
	// states[0].traj[5].comp[0].base.addKnot(0, .2);
	
	// states[0].traj.emplace_back(front_shoulder, back_shoulder); //swing shoulder
	// states[0].traj[6].comp.emplace_back();
	// states[0].traj[6].comp[0].base.addKnot(0, -.1);
	
	// states[0].traj.emplace_back(back_elbow, front_elbow); //stance elbow
	// states[0].traj[7].comp.emplace_back();
	// states[0].traj[7].comp[0].base.addKnot(0, .1);
	
	// states[0].traj.emplace_back(front_elbow, back_elbow); //swing elbow
	// states[0].traj[8].comp.emplace_back();
	// states[0].traj[8].comp[0].base.addKnot(0, 0);
	
	// states[0].traj.emplace_back(neck, neck); //neck
	
	// states[0].traj.emplace_back(-1, -1); //root
	// states[0].traj[10].comp.emplace_back();
	// states[0].traj[10].comp[0].base.addKnot(0, 0);
}

void Controller::loadFSM(std::string filename)
{
	std::ifstream file(filename.c_str());
	std::string word;
	if (!file.fail())
	{
		int a;
		double b;
		bool c, d, e, f;
		file >> a >> b >> c >> d >> e >> f;
		states.emplace_back(a, b, c, d, e, f);
	}
	while (file >> word)
	{
		if (word == "trajectory")
		{
			int a, b;
			file >> a >> b;
			states.back().traj.emplace_back(a, b);
		}
		if (word == "strength")
		{
			double a, b;
			file >> a >> b;
			states.back().traj.back().strength.addKnot(a, b);
		}
		if (word == "component")
			states.back().traj.back().comp.emplace_back();
		if (word == "base")
		{
			double a, b;
			file >> a >> b;
			states.back().traj.back().comp.back().base.addKnot(a, b);
		}
		if (word == "vscale")
		{
			double a, b;
			file >> a >> b;
			states.back().traj.back().comp.back().vscale.addKnot(a, b);
		}
	}
}

void Controller::Step(double dt)
{
	//we should really do something if these fail
	if (SwRay.cast());
	if (StRay.cast());
	setStanceSwing();
	updateCoM();
	computeDesired();
	//set upper body pose?
	//adjustHeight();
	computeTorques();
	applyTorques(dt);
}

void Controller::checkTransition(double dt)
{
	if (advanceTime(dt) != -1)
	{
		setStanceSwing();
		updateCoM();
		SwFootStart = ((RevJoint*)SwAnkle)->GetAnchor();
		//do stuff with heightTraj?
		swingTraj.clear();
		swingTraj.addKnot(0, 0);
		swingTraj.addKnot(1, 0);
	}
	
	SwRay.origin.Set(SwFoot->position.x + .5 - std::abs(phi-.5), SwFoot->position.y);
	StRay.origin = StFoot->position;
}

int Controller::advanceTime(double dt)	//needs to check and see if it's fallen
{
	if(FSMStateIndex < 0)
		FSMStateIndex = 0;
	
	if(dt <= 0)
		return -1;
	
	if (FSMStateIndex >= states.size())
		return -1;
	
	/** check for body contact here **/
	
	phi += dt / states[FSMStateIndex].stime;
	
	//will need to account for up direction eventually
	bool contacted = SwFoot->contactForce.y > .005;
	bool footTrans = contacted and states[FSMStateIndex].transonfoot and phi > .5;
	
	if (phi >= 1 or footTrans)
	{
		if (states[FSMStateIndex].nextstate < 0)
			FSMStateIndex = -1;
		else if(states[FSMStateIndex].nextstate >= states.size())
			FSMStateIndex = states.size() - 1;
		else
			FSMStateIndex = states[FSMStateIndex].nextstate;
		
		if (states[FSMStateIndex].reverse)
			stance = !stance;
		else if (!states[FSMStateIndex].keep)
			stance = states[FSMStateIndex].setstance;
		
		phi = 0;
		return states[FSMStateIndex].nextstate;
	}
	
	return -1;
}

void Controller::computeTorques()		//might need tweaks
{	
	desiredPose.zero();
	for (int i = 0; i < controlParams.size(); i++)
		controlParams[i].controlled = true;
	
	desiredroot = 0;
	
	double phiToUse = phi;
	if (phiToUse > 1)
		phiToUse = 1;
	
	State* curState = &states[FSMStateIndex];
	
	for (int i = 0; i < curState->traj.size(); i++)
	{
		int jIndex = curState->traj[i].getJoint(stance);
		Vector2D d0, v0;
		
		computeD0(phiToUse, &d0);
		computeV0(phiToUse, &v0);
		double neworient = curState->traj[i].evaluate(phiToUse, StancetoCoM-d0, 
			man->comPosition-v0);
		
		if (jIndex == -1)
		{
			desiredroot = neworient;
			rootControlParams.strength = curState->traj[i].evaluateStrength(phiToUse);
		}
		else
		{
			desiredPose.joints[jIndex].orient = neworient;
			controlParams[jIndex].strength = curState->traj[i].evaluateStrength(phiToUse);
		}
	}
	
	if (!doubleStance)
		computeIKSwingTargets(0.001);
	
	computePDTorques();
	
	//bubble up torques(?)
	
	compensateGravity();
	
	rootTorque = 0;
	if (StFoot->contact)
		COMJT();

	if (doubleStance)
		computeLegTorques();
	
	computeHipTorques();
}

void Controller::applyTorques(double dt)
{
	bool noOldTorqueInfo = false;
	if (oldtorques.size() != torques.size())
	{
		oldtorques.clear();
		noOldTorqueInfo = true;
	}

	double tmpTorque, deltaT;
	for (int i = 0; i < man->joint.size(); i++)
	{
		if (noOldTorqueInfo)
			oldtorques.emplace_back(torques[i]);
		
		deltaT = torques[i] - oldtorques[i];
		
		deltaT = Clamp(deltaT, -.8, .8);
		
		tmpTorque = oldtorques[i] + deltaT;
		
		//((RevJoint*)man->joint[i])->SetMotor(true, 100, tmpTorque);
		((RevJoint*)man->joint[i])->ApplyTorque(tmpTorque, dt);
		oldtorques[i] = tmpTorque;
	}
}

void Controller::computeIKSwingTargets(double dt)
{
	Vector2D pnow = getSwingTarget(phi, man->comPosition);
	Vector2D pfuture = getSwingTarget(std::min(phi + dt, 1.0), 
		man->comPosition + man->comVelocity * dt);
	
	double hipangle, kneeangle;
	elbow = 
	IKJoint::TwoLinkIKSolve((RevJoint*)SwHip, (RevJoint*)SwKnee, 
		((RevJoint*)SwAnkle)->GetAnchor(), pnow, hipangle, kneeangle);
	
	endpoint = pnow;
	
	desiredPose.joints[stance ? front_hip : back_hip].orient = hipangle;
	desiredPose.joints[stance ? front_knee : back_knee].orient = kneeangle;
	
	//compute angular velocities - STILL POSSIBLY BROKEN
	double hipfuture, kneefuture, hip, knee;
	IKJoint::TwoLinkIKSolve((RevJoint*)SwHip, (RevJoint*)SwKnee,
		((RevJoint*)SwAnkle)->GetAnchor(), pfuture - man->body[torso]->velocity * dt, 
		hipfuture, kneefuture);
	double diff = hipfuture - hipangle;
	hip = diff / dt;
	hip -= man->body[torso]->angVel;
	diff = kneefuture - kneeangle;
	knee = diff / dt;
	desiredPose.joints[stance ? front_hip : back_hip].angvel = hip;
	desiredPose.joints[stance ? front_knee : back_knee].angvel = knee;
}

Vector2D Controller::getSwingTarget(double t, Vector2D com)
{
	Vector2D step;
	step.x = swingTraj.evaluate_catmull_rom(t);
	step.x += com.x;
	
	double height = 0;
	height = (SwRay.origin + SwRay.direction * SwRay.result * SwRay.length).y;
	double panic = -4 * phi * phi + 4 * phi;
	panic *= .05;
	
	step.y = heightTraj.evaluate_catmull_rom(t) + height + panic;
	//compute delta???
	return step;
}

void Controller::computePDTorques()
{
	for (int i = 0; i < controlParams.size(); i++)
	{
		if (controlParams[i].controlled)
		{
			double curAngle = ((RevJoint*)man->joint[i])->GetAngle();
			double curAngVel = ((RevJoint*)man->joint[i])->GetAngVel();
			
			double desAngle = desiredPose.joints[i].orient;
			double desAngVel = desiredPose.joints[i].angvel;
			
			if (man->joint[i] == SwAnkle)
			{
				curAngle = SwFoot->orient;
				curAngVel = SwFoot->angVel;
				desAngle += (angleV(SwRay.norm) - pi/2);
			}
			if (man->joint[i] == StAnkle)
			{
				curAngle = StFoot->orient;
				curAngVel = StFoot->angVel;
				desAngle += (angleV(StRay.norm) - pi/2);
			}
			double torque = (desAngle - curAngle) * controlParams[i].kp;
			torque += (desAngVel - curAngVel) * controlParams[i].kd;
			torque *= controlParams[i].strength;
			
			torque = Clamp(torque, -controlParams[i].maxAbsTorque,
				controlParams[i].maxAbsTorque);
			
			torques[i] = torque;
		}
		else
		{
			torques[i] = 0;
		}
	}
}

void Controller::compensateGravity()
{
	Vector2D Force, tmpV;
	double tmpT;
	Force.Set(0, SwFoot->mass.m * 9.8);
	tmpV = SwFoot->position - ((RevJoint*)SwAnkle)->GetAnchor();
	tmpT = cross(tmpV, Force);
	torques[stance ? front_ankle : back_ankle] += tmpT;
	tmpV = SwCalf->position - ((RevJoint*)SwKnee)->GetAnchor();
	tmpT = cross(tmpV, Force);
	torques[stance ? front_knee : back_knee] += tmpT;
	tmpV = SwThigh->position - ((RevJoint*)SwHip)->GetAnchor();
	tmpT = cross(tmpV, Force);
	torques[stance ? front_hip : back_hip] += tmpT;
	
	Force.Set(0, SwCalf->mass.m * 9.8);
	tmpV = SwCalf->position - ((RevJoint*)SwKnee)->GetAnchor();
	tmpT = cross(tmpV, Force);
	torques[stance ? front_knee : back_knee] += tmpT;
	tmpV = SwThigh->position - ((RevJoint*)SwHip)->GetAnchor();
	tmpT = cross(tmpV, Force);
	torques[stance ? front_hip : back_hip] += tmpT;
	
	Force.Set(0, SwThigh->mass.m * 9.8);
	tmpV = SwThigh->position - ((RevJoint*)SwHip)->GetAnchor();
	tmpT = cross(tmpV, Force);
	torques[stance ? front_hip : back_hip] += tmpT;
	
	Force.Set(0, man->body[front_forearm]->mass.m * 9.8);
	tmpV = man->body[front_forearm]->position - 
		((RevJoint*)man->joint[front_elbow])->GetAnchor();
	tmpT = cross(tmpV, Force);
	torques[front_elbow] += tmpT;
	tmpV = man->body[front_arm]->position - 
		((RevJoint*)man->joint[front_shoulder])->GetAnchor();
	tmpT = cross(tmpV, Force);
	torques[front_shoulder] += tmpT;
	
	Force.Set(0, man->body[front_arm]->mass.m * 9.8);
	tmpV = man->body[front_arm]->position - 
		((RevJoint*)man->joint[front_shoulder])->GetAnchor();
	tmpT = cross(tmpV, Force);
	torques[front_shoulder] += tmpT;
	
	Force.Set(0, man->body[back_forearm]->mass.m * 9.8);
	tmpV = man->body[back_forearm]->position - 
		((RevJoint*)man->joint[back_elbow])->GetAnchor();
	tmpT = cross(tmpV, Force);
	torques[back_elbow] += tmpT;
	tmpV = man->body[back_arm]->position - 
		((RevJoint*)man->joint[back_shoulder])->GetAnchor();
	tmpT = cross(tmpV, Force);
	torques[back_shoulder] += tmpT;
	
	Force.Set(0, man->body[back_arm]->mass.m * 9.8);
	tmpV = man->body[back_arm]->position - 
		((RevJoint*)man->joint[back_shoulder])->GetAnchor();
	tmpT = cross(tmpV, Force);
	torques[back_shoulder] += tmpT;
}

void Controller::COMJT()			//has trouble keeping toes on ground
{
	double m = man->body[torso]->mass.m + StThigh->mass.m + StCalf->mass.m;
	Vector2D anklePos = ((RevJoint*)StAnkle)->GetAnchor();
	Vector2D kneePos = ((RevJoint*)StKnee)->GetAnchor();
	Vector2D hipPos = ((RevJoint*)StHip)->GetAnchor();
	
	Vector2D desVel((desiredvelocity - man->comVelocity.x)*10, 0);
	double mass = 0;
	for (int i = 0; i < man->body.size(); i++)
		mass += man->body[i]->mass.m;
	Vector2D fA = desVel * mass;
	fA.x = Clamp(fA.x, -60, 60);
	
	// man->body[torso]->ApplyForce(fA);
	// return;
	
	Vector2D f1 = (StCalf->position - anklePos) * StCalf->mass.m 
				+ (StThigh->position - anklePos) * StThigh->mass.m
				+ (man->body[torso]->position - anklePos) * man->body[torso]->mass.m;
	f1 /= m;
	
	Vector2D f2 = (StThigh->position - kneePos) * StThigh->mass.m
				+ (man->body[torso]->position - kneePos) * man->body[torso]->mass.m;
	f2 /= m;
	
	Vector2D f3 = (man->body[torso]->position - hipPos) * man->body[torso]->mass.m;
	f3 /= m;
	
	double ankletorque = cross(f1, fA);
	//preprocess
	if ((StFoot->angVel < -0.2 && ankletorque > 0) || (StFoot->angVel > 0.2 && ankletorque < 0)
		|| std::abs(StFoot->angVel) > 1)
		ankletorque = 0;
	ankletorque = Clamp(ankletorque, -.4, .4);					//questionable
	torques[stance ? back_ankle : front_ankle] -= ankletorque;
	torques[stance ? back_knee : front_knee] -= cross(f2, fA);
	torques[stance ? back_hip : front_hip] -= cross(f3, fA);
	rootTorque += cross(f3, fA);
}

void Controller::computeLegTorques()		//doesn't get used, apparently
{
	Vector2D desVel((desiredvelocity - man->comVelocity.x) * 30, 0);
	double mass = 0;
	for (int i = 0; i < man->body.size(); i++)
		mass += man->body[i]->mass.m;
	Vector2D fA = desVel * mass;
	fA.x = Clamp(fA.x, -60, 60);
	
	Vector2D r = man->comPosition - ((RevJoint*)SwAnkle)->GetAnchor();
	double ankletorque = cross(r, fA);
	//preprocess?
	torques[stance ? front_ankle : back_ankle] += ankletorque;
	r = man->comPosition - ((RevJoint*)SwKnee)->GetAnchor();
	torques[stance ? front_knee : back_knee] += cross(r, fA);
	r = man->comPosition - ((RevJoint*)SwHip)->GetAnchor();
	torques[stance ? front_hip : back_hip] += cross(r, fA);
	
	rootTorque = -cross(r, fA);
}

void Controller::computeHipTorques()
{
	Vector2D stanceForce = StFoot->contactForce;
	Vector2D swingForce = SwFoot->contactForce;
	double totalF = (stanceForce + swingForce).y;	//needs to account for variable up
	
	double StSwRatio = -1;
	if (!Equal(totalF, 0))
		StSwRatio = stanceForce.y / totalF;
	
	if (StSwRatio < 0)
		rootControlParams.strength = 0;
	
	double rootStrength = rootControlParams.strength;
	rootStrength = Clamp(rootStrength, 0, 1);
	rootControlParams.strength = 1;
	
	double curAngle = man->body[torso]->orient;
	double curAngVel = man->body[torso]->angVel;
	
	double desAngle = desiredroot;
	double desAngVel = 0;
	
	double torque = (desAngle - curAngle) * rootControlParams.kp;
	torque += (desAngVel - curAngVel) * rootControlParams.kd;
	torque *= rootControlParams.strength;
	
	torque = Clamp(torque, -rootControlParams.maxAbsTorque, rootControlParams.maxAbsTorque);
	
	torque += rootTorque;
	
	double SwHipTorque = torques[stance ? front_hip : back_hip];
	double StHipTorque = torques[stance ? back_hip : front_hip];
	
	double makeupTorque = 0;
	makeupTorque -= torques[front_hip];
	makeupTorque -= torques[back_hip];
	makeupTorque -= torque;
	
	StHipTorque += makeupTorque * StSwRatio * rootStrength;
	SwHipTorque += makeupTorque * (1-StSwRatio) * rootStrength;
	
	double SwMax = controlParams[stance ? front_hip : back_hip].maxAbsTorque;
	double StMax = controlParams[stance ? back_hip : front_hip].maxAbsTorque;
	SwHipTorque = Clamp(SwHipTorque, -SwMax, SwMax);
	StHipTorque = Clamp(StHipTorque, -StMax, StMax);
	
	torques[stance ? front_hip : back_hip] = SwHipTorque;
	torques[stance ? back_hip : front_hip] = StHipTorque;
}

void Controller::setStanceSwing()
{
	SwHip   = man->joint[stance ? front_hip   : back_hip   ];
	SwKnee  = man->joint[stance ? front_knee  : back_knee  ];
	SwAnkle = man->joint[stance ? front_ankle : back_ankle ];
	StHip   = man->joint[stance ? back_hip    : front_hip  ];
	StKnee  = man->joint[stance ? back_knee   : front_knee ];
	StAnkle = man->joint[stance ? back_ankle  : front_ankle];
	SwThigh = man->body [stance ? front_thigh : back_thigh ];
	SwCalf  = man->body [stance ? front_calf  : back_calf  ];
	SwFoot  = man->body [stance ? front_foot  : back_foot  ];
	StThigh = man->body [stance ? back_thigh  : front_thigh];
	StCalf  = man->body [stance ? back_calf   : front_calf ];
	StFoot  = man->body [stance ? back_foot   : front_foot ];
}

Vector2D Controller::computeIP()
{
	Vector2D step;
	Vector2D vel = man->comVelocity;
	double h = fabs(man->comPosition.y - StFoot->position.y);
	step.x = vel.x * sqrt(h / 9.8 + vel.x * vel.x / (4 * 9.8 * 9.8)) * 1.1;
	step.y = 0;
	return step;
}

Vector2D Controller::computeEstimate(Vector2D comPos, double phase)
{
	Vector2D step = computeIP();
	
	step.x -= desiredvelocity / 20;
	step.x = Clamp(step.x, -.6, .6);
	
	Vector2D initialstep = SwFootStart - comPos;
	double t = 1 - phase;
	t *= t;
	t = Clamp(t, 0, 1);
	Vector2D ans = step * (1-t);
	ans += initialstep * t;
	
	ans.y = 0;
	
	return ans;
}

void Controller::computeDesired()
{
	Vector2D step = computeEstimate(man->comPosition, phi);
	swingTraj.setKnotValue(0, step.x);
	
	double dt = 0.001;
	step = computeEstimate(man->comPosition + man->comVelocity * dt, phi + dt);
	swingTraj.setKnotValue(1, step.x);
	
	swingTraj.setKnotPosition(0, phi);
	swingTraj.setKnotPosition(1, phi + dt);
}

void Controller::updateCoM()
{
	man->updateCoM();
	StancetoCoM = man->comPosition - StFoot->position;
	Vector2D feetMidpoint = (StFoot->position + SwFoot->position);
	feetMidpoint /= 2.0;
	dsCoMErr = feetMidpoint - man->comPosition;
	dsCoMErr.y = 0;
}

