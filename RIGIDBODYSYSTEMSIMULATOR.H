#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "DirectXMath.h"
#include <optional>
#include <functional>
#include "../../Simulations/util/matrixbase.h"
#include "collisionDetect.h"

//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

	// MY FUNC
	Mat4 calculateTensor(const std::tuple<Vec3, Vec3, int>& rigidBody);
	static Vec3 crossproduct(const Vec3& V1, const Vec3& V2);
	void init();
	Vec3 calculateImpulse(int bodyA, int bodyB, CollisionInfo& info);
	void simulateWorld(float timeStep);
private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;

	// MY ATTRIB
	// 
	// Position, Size, Mass
	std::vector<std::tuple<Vec3, Vec3, int>> rigidBodies;
	std::vector<Quat> orientations;
	std::vector<Vec3> velocities;
	std::vector<Vec3> angularVelocities;
	std::vector<Vec3> angularMomentum;
	std::vector<Vec3> torques;
	std::vector<Mat4> tensors;
	std::vector<int> masses;
	bool isInit = false;

	std::vector<Vec3> forces;
	std::vector<float>bounciness;



	// UI Attributes
	float timestep{0.1};
	bool gravityToggle;

	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif