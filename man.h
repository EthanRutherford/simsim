#ifndef MAN_H
#define MAN_H

#include "body.h"
#include "joint.h"
#include <vector>
#include <fstream>

enum Bodies{
	back_forearm, back_arm, back_foot, back_calf, back_thigh, torso,
	head, front_thigh, front_calf, front_foot, front_arm, front_forearm
};
enum Joints{
	back_shoulder, front_shoulder, back_elbow, front_elbow, neck,
	back_hip, front_hip, back_knee, front_knee, back_ankle, front_ankle
};

class Controller;
class Man{
	friend class Controller;
	public:
		void Initialize(double x, double y);
		Body* RegisterBodies();
		Joint* RegisterJoints();
		Vector2D GetPos();
	private:
		std::vector<Body*> body;
		std::vector<Joint*> joint;
		int b, j;
		Vector2D comPosition;
		Vector2D comVelocity;
		
		void updateCoM();
};

struct TrajComp{
	TrajComp() : offset(0) {}
	trajectory1D base;
	trajectory1D dscale;
	trajectory1D vscale;
	double offset;
	double compute(double phi, Vector2D d, Vector2D v){
		double baseAngle = offset;
		double scale = 1;
		if (dscale.getKnotCount() > 0)
			scale *= dscale.evaluate_linear(d.x);
		if (vscale.getKnotCount() > 0)
			scale *= vscale.evaluate_linear(v.x);
		if (base.getKnotCount() > 0)
			baseAngle += base.evaluate_catmull_rom(phi) * scale;
		return baseAngle;
	};
};

struct Trajectory{
	Trajectory() {}
	Trajectory(int l, int r) : leftstance(l), rightstance(r) {}
	int getJoint(bool s) { return s ? leftstance : rightstance; }
	std::vector<TrajComp> comp;
	trajectory1D strength;
	int leftstance;
	int rightstance;
	double evaluate(double phi, Vector2D d, Vector2D v){
		double q = 0;
		for (int i = 0; i < comp.size(); i++)
			q += comp[i].compute(phi, d, v);
		return q;
	}
	double evaluateStrength(double phi){
		if (strength.getKnotCount() == 0) return 1.0;
		return strength.evaluate_catmull_rom(phi);
	}
};

struct State{
	State() {}
	State(int n, double t, bool r, bool k, bool s, bool f) : 
		nextstate(n), stime(t), reverse(r), keep(k), setstance(s), transonfoot(f) {}
	std::vector<Trajectory> traj;
	int nextstate;
	double stime;
	bool reverse;
	bool keep;
	bool setstance;
	bool transonfoot;
	trajectory1D dTraj;
	trajectory1D vTraj;
};

struct ControlParams{
	ControlParams() {}
	ControlParams(Joint* j, double p, double d, double m) :
		joint(j), controlled(false), kp(p), kd(d), maxAbsTorque(m), strength(1) {}
	Joint* joint;
	bool controlled;
	double kp, kd;
	double maxAbsTorque;
	double strength;
};

struct PoseJoint{
	double orient;
	double angvel;
};

struct Pose{
	std::vector<PoseJoint> joints;
	void zero() {
		for (PoseJoint &a : joints)
		{
			a.orient = 0;
			a.angvel = 0;
		}
	}
};

class Controller{
	public:
		Controller(Man* m);
		void loadFSM(std::string filename);
		void Step(double dt);
		void checkTransition(double dt);
		int advanceTime(double dt);
		void computeTorques();
		void applyTorques(double dt);
		void computeIKSwingTargets(double dt);
		Vector2D getSwingTarget(double t, Vector2D com);
		void computePDTorques();
		void compensateGravity();
		void COMJT();
		void computeLegTorques();
		void computeHipTorques();
		void setStanceSwing();
		Vector2D computeIP();
		Vector2D computeEstimate(Vector2D comPos, double phase);
		void computeDesired();
		void updateCoM();
		
		mRay SwRay, StRay;
		
		Vector2D endpoint, elbow;
		void SetSpeed(double speed) {desiredvelocity = speed;}
		
	private:
		Man* man;
		std::vector<double> torques;
		std::vector<double> oldtorques;
		std::vector<State> states;
		std::vector<ControlParams> controlParams;
		ControlParams rootControlParams;
		
		Pose desiredPose;
		double desiredroot;
		double rootTorque;
		
		bool stance;
		bool doubleStance;
		int FSMStateIndex;
		Joint* SwHip,* SwKnee,* SwAnkle,* StHip,* StKnee,* StAnkle;
		Body* SwThigh,* SwCalf,* SwFoot,* StThigh,* StCalf,* StFoot;
		Vector2D SwFootStart;
		Vector2D StancetoCoM;
		Vector2D dsCoMErr;
		double phi;
		
		double desiredvelocity;
		trajectory1D swingTraj;
		trajectory1D heightTraj;
		
		inline void computeD0(double phi, Vector2D* d0) {
			State* curState = &states[FSMStateIndex];
			computeDorV(phi, &curState->dTraj, d0);
		}
		inline void computeV0(double phi, Vector2D* v0) {
			State* curState = &states[FSMStateIndex];
			computeDorV(phi, &curState->vTraj, v0);
		}
		inline static void computeDorV(double phi, trajectory1D* traj, Vector2D* result) {
			result->y = 0;
			if(traj->getKnotCount() == 0)
				result->x = 0;
			else
				result->x = traj->evaluate_catmull_rom(phi);
		}
};

#endif