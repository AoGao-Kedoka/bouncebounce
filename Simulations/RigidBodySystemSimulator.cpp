#include "RigidBodySystemSimulator.h"

#define DF_POS {0, 0, 0}
#define DF_SIZE {1, 0.6, 0.5}
#define DF_M 2.0

//----------------------------------------
// Constructor and Destructor
//----------------------------------------
RigidBodySystemSimulator::RigidBodySystemSimulator():
	m_externalForce(Vec3{ 1, 1, 0 }), m_forcePosition(Vec3{0.3, 0.3, 0.25}) {
	this->reset();
	if (m_iTestCase != TESTCASEUSEDTORUNTEST)
		this->buildRigid(1);
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
		std::cout << "Demo for test usage" << std::endl;
		break;
	case 1:
		this->buildRigid(1);
		std::cout << "Demo1" << std::endl;
		break;
	case 2:
		this->buildRigid(1);
		std::cout << "Demo2" << std::endl;
		break;
	case 3:
		this->buildRigid(2);
		std::cout << "Demo3" << std::endl;
		break;
	case 4:
		this->buildRigid(10);
		std::cout << "Demo4" << std::endl;
	}
	
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed){
	//TODO: may be still needed, who knows
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep){
	for (const auto& r : rigids) {
		this->simulateRigid(r);
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
	r.velocity = Vec3(0, 0, 0);
	r.orientation = Quat(0);
	rigids.push_back(r);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation){
	this->rigids.at(i).orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity){
	this->rigids.at(i).velocity = velocity;
}


int RigidBodySystemSimulator::getNumberOfRigidBodies(){ return rigids.size(); }

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i){ return rigids[i].center; }

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i){
	//TODO
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i){
	//TODO
	return Vec3();
}

//----------------------------------------
// Helper Functions
//----------------------------------------
void RigidBodySystemSimulator::simulateRigid(Rigidbody r){
	//TODO
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
