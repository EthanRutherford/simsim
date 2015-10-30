#ifndef JOINT_H
#define JOINT_H

#include "body.h"

class IKJoint;

class Joint{
	public:
		Joint(Body* a, Body* b) : A(a), B(b) {}
		virtual void Initialize(double dt) = 0;
		virtual void ApplyImpulse(double dt) = 0;
		virtual void PositionalCorrection() = 0;
	protected:
		Body* A;
		Body* B;
};

class RopeJoint : public Joint{
	public:
		RopeJoint(Body* a, Body* b, Vector2D c, Vector2D d, double e) : Joint(a, b), maxDist(e)
		{
			AnchorA = c - a->mass.CoM;
			AnchorB = d - b->mass.CoM;
		}
		void Initialize(double dt);
		void ApplyImpulse(double dt);
		void PositionalCorrection();
	private:
		Vector2D AnchorA;
		Vector2D AnchorB;
		Vector2D u;
		double maxDist;
		double mass;
		double impulse;
		double dist;
};

class RevJoint : public Joint{
	friend class IKJoint;
	public:
		RevJoint(Body* a, Body* b, Vector2D c, Vector2D d) : Joint(a, b) {
			AnchorA = c - a->mass.CoM;
			AnchorB = d - b->mass.CoM;
			refAngle = B->orient - A->orient;
			motorEnabled = false;
			motorImpulse = 0;
			limitEnabled = false;
			cumImpulse.Set(0,0,0);
		}
		void Initialize(double dt);
		void ApplyImpulse(double dt);
		void PositionalCorrection();
		void SetMotor(bool on, double speed, double tlimit);
		void ApplyTorque(double torque, double dt);
		void SetLimit(bool on, double upper, double lower);
		double GetAngle();
		double GetAffineAngle();
		double GetAngVel();
		Vector2D GetAnchor();
	private:
		Vector2D AnchorA;
		Vector2D AnchorB;
		Matrix3D mass;
		
		bool motorEnabled;
		double motorSpeed;
		double motorMaxTorque;
		double motorImpulse;
		
		bool limitEnabled;
		Angle lowerLimit;
		Angle upperLimit;
		Angle refAngle;
		Vector3D cumImpulse;
		
		enum State{
			_atUpper,
			_atLower,
			_between
		};
		State state;
};

class WheelJoint : public Joint{
	public:
		WheelJoint(Body* a, Body* b, Vector2D c, Vector2D d, Vector2D e, double f, double g) :
			Joint(a, b), AxisFrdm(e), springFreq(f), springDamp(g) 
		{
			AnchorA = c - a->mass.CoM;
			AnchorB = d - b->mass.CoM;
			bool motorEnabled = false;
			motorImpulse = 0;
			springImpulse = 0;
			cumImpulse = 0;
		}
		void Initialize(double dt);
		void ApplyImpulse(double dt);
		void PositionalCorrection();
		void SetMotor(bool on, double speed, double tlimit);
		void SetMotorSpeed(double speed) {motorSpeed = speed;}
	private:
		Vector2D AnchorA;
		Vector2D AnchorB;
		Vector2D AxisFrdm;
		Vector2D ax, ay;
		double sAx, sAy;
		double sBx, sBy;
		bool motorEnabled;
		double motorSpeed;
		double motorMaxTorque;
		double motorImpulse, springImpulse, cumImpulse;
		double springFreq;
		double springDamp;
		double mass;
		double smass, bias, gamma;
};

class IKJoint{
	public:
		static Vector2D TwoLinkIKSolve(RevJoint* Parent, RevJoint* Child, Vector2D endpoint, 
			Vector2D target, double& p, double& c);
		IKJoint(RevJoint* j, Vector2D endPoint);
		void AddJoint(RevJoint* j);
		void SetTarget(Vector2D t);
		void SetTarget(bool on);
		void Solve();
	private:
		void solve(int i);
		bool targeted;
		Vector2D effector;
		Vector2D effRel;
		Vector2D target;
		std::vector<RevJoint*> joints;
};

#endif