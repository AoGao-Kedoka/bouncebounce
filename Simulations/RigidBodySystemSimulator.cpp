#include "RigidBodySystemSimulator.h"

#define DF_POS {0, 0, 0}
#define DF_SIZE {1, 0.6, 0.5}
#define DF_M 2.0

static int counter = 0;

//----------------------------------------
// Constructor and Destructor
//----------------------------------------
RigidBodySystemSimulator::RigidBodySystemSimulator():
	m_externalForce(Vec3{ 1, 1, 0 }), m_forcePosition(Vec3{0.3, 0.3, 0.25}) {
	this->reset();
}

RigidBodySystemSimulator::~RigidBodySystemSimulator() {
	this->rigids.clear();
}


//----------------------------------------
// Functions
//----------------------------------------
const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "DemoForTest,\
			Demo1: A simple one-step test,\
			Demo2: Simple single body simulation,\
			Demo3: Two-rigid-body collistion scene,\
			Demo4: Complex simulation";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset(){
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	int bodNum = rigids.size();
	this->buildRigid(bodNum);
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext){
	for (const auto& r : rigids) {
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		// using DirectX function to change vectors to matrices
		XMMATRIX scaleM = XMMatrixScalingFromVector(r.size.toDirectXVector());
		XMMATRIX rotM = r.orientation.getRotMat().toDirectXMatrix();
		XMMATRIX transM = XMMatrixTranslationFromVector(r.center.toDirectXVector());
		DUC->drawRigidBody(scaleM * rotM * transM);
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase){
	m_iTestCase = testCase;
	switch (m_iTestCase) {
	case 0:
		counter = 0;
		std::cout << "Demo for test usage" << std::endl;
		break;
	case 1:
		counter = 0;
		this->buildRigid(1);
		std::cout << "Demo1" << std::endl;
		break;
	case 2:
		counter = 0;
		this->buildRigid(1);
		std::cout << "Demo2" << std::endl;
		break;
	case 3:
		counter = 0;
		this->buildRigid(2);
		std::cout << "Demo3" << std::endl;
		break;
	case 4:
		counter = 0;
		this->buildRigid(10);
		std::cout << "Demo4" << std::endl;
	}
	
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed){
	//TODO: may be still needed, who knows
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep){
	if (m_iTestCase == 1) {
		if (counter == 0)
			this->simulateRigid(0.01);
	}
	else {
		this->simulateRigid(0.01);
	}
}

void RigidBodySystemSimulator::onClick(int x, int y){
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y){
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

//----------------------------------------
// Exra Functions
//----------------------------------------
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force){
	//TODO
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass){
	struct Rigidbody r;
	r.center = position;
	r.size = size;
	r.mass = mass;
	r.l_velocity = Vec3(0, 0, 0);
	r.a_velocity = Vec3(0, 0, 0);
	r.a_momentum = Vec3(0, 0, 0);
	// rotation z axis -90 degrees 
	// source: https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm
	r.orientation = Quat(0.71,0.71,0,0);
	rigids.push_back(r);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation){
	this->rigids.at(i).orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity){
	this->rigids.at(i).l_velocity = velocity;
}


int RigidBodySystemSimulator::getNumberOfRigidBodies(){ return rigids.size(); }

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i){ return rigids[i].center; }

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i){
	return this->rigids.at(i).l_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i){
	return this->rigids.at(i).a_velocity;
}

//----------------------------------------
// Helper Functions
//----------------------------------------
void RigidBodySystemSimulator::simulateRigid(float timeStep){
	for (auto& r : rigids) {
		// simulate roatation
		if (counter == 0) {
			/*
				precomute if it's the first iteration
				formel from wikipedia: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
				Matrix should look like this		
					| iw	0	0  |
				I =	| 0		ih	0  |
					| 0		0	id |
			*/
			float iw = r.mass * (pow(r.size.z, 2) + pow(r.size.y, 2)) / 12;
			float ih = r.mass * (pow(r.size.x, 2) + pow(r.size.z, 2)) / 12;
			float id = r.mass * (pow(r.size.x, 2) + pow(r.size.y, 2)) / 12;
			auto i0 = XMMatrixScaling(iw, ih, id);
			auto inverse_i = XMMatrixInverse(nullptr, i0);
			r.i_tensor = inverse_i;
			++counter;
		}
		// calculate torque q: xi X fi
		Vec3 q = crossProduct(m_externalForce, m_forcePosition);
		
		// integrate the orientation r using the angular velocity
		Quat tmp = (0, r.a_velocity.x, r.a_velocity.y, r.a_velocity.z);
		r.orientation = timeStep * 0.5 * quatMul(tmp, r.orientation);
		r.orientation /= r.orientation.norm();

		// integrate angular momentum
		r.a_momentum += timeStep * q;

		// update Inertia Tensor
		r.i_tensor= r.orientation.getRotMat().toDirectXMatrix() * 
					r.i_tensor * XMMatrixTranspose(r.orientation.getRotMat().toDirectXMatrix());

		// update angular velocity, no need to update all points because it will be handled in drawing
		XMVECTOR angular = XMVector3Transform(r.a_momentum.toDirectXVector(), r.i_tensor);
		r.a_velocity = angular;
		std::cout << "Angular Velocity: " << r.a_velocity << std::endl;

		// simulate translation with euler
		r.center += timeStep * r.l_velocity;
		Vec3 acc = m_externalForce / r.mass;
		r.l_velocity += timeStep * acc;
		std::cout << "Linear Velocity: " << r.l_velocity << std::endl;
		std::cout << "Position: " << r.center << std::endl;
	}
}

void RigidBodySystemSimulator::buildRigid(int i) {
	this->rigids.clear();
	if (i == 1) {
		this->addRigidBody(DF_POS, DF_SIZE, DF_M);
	}
	else {
		//TODO
	}
}

Vec3 RigidBodySystemSimulator::crossProduct(Vec3 a, Vec3 b) {
	Vec3  ret;
	ret.x = a.y * b.z - a.z * b.y;
	ret.y = a.z * b.x - a.x * b.z;
	ret.z = a.x * b.y - a.y * b.x;
	return ret;
}

float RigidBodySystemSimulator::dotProduct(Vec3 a, Vec3 b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

Quat RigidBodySystemSimulator::quatMul(Quat a, Quat b) {
	// perform quaternion multiplication
	// S = S1 * S2 - v2 * v2
	// V = S1 * v2 + S2 * v1 + v1 x v2
	Vec3 v1 = Vec3(a.y, a.z, a.w);
	Vec3 v2 = Vec3(b.y, b.z, b.w);
	float scalar = a.x * b.x - dotProduct(v1, v2);
	Vec3 vector = a.x * v2 + b.x * v1 + crossProduct(v1, v2);
	Quat ret;
	ret.x = scalar, ret.y = vector.x, ret.z = vector.y, ret.w = vector.z;
	return ret;
}


