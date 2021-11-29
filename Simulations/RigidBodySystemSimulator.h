#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "collisionDetect.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 


#define TESTCASEUSEDTORUNTEST 2

struct RigidBodyBox {
	RigidBodyBox(Vec3 position, Vec3 dimensions, int mass):
		position(position), 
		width(dimensions.x), depth(dimensions.y), height(dimensions.z), 
		mass(mass), diagInertiaInv(0.,0.,0.) {}

	Vec3 position;
	Vec3 linearVelocity;
	Vec3 angularVelocity; 
	double width;
	double depth;
	double height;
	Vec3 angularMomentum;
	double mass;
	Quat orientation;
	Vec3 totalForce; // acting on center of mass
	bool isFixed = false;
	Mat4 worldMatrix;
	bool canCollide = true;
	Vec3 diagInertiaInv;
};

struct force {
	// f force applied at point p on ith body.
	force(Vec3 f, Vec3 p, int i): f(f), p(p), i(i) {}

	Vec3 f;
	Vec3 p;
	int i;
};

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void calcImpulse(CollisionInfo info, RigidBodyBox& rb1, RigidBodyBox& rb2, int c);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);


private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_CmPosition;
	Vec3 m_CmVelocity;

	Vec3 m_fGravity;
	// for UI interaction
	float m_fGravityMult;
	Vec3 m_externalForce;
	std::vector<RigidBodyBox> Boxes;
	std::vector<force> forces;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif