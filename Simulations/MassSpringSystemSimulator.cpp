#include "MassSpringSystemSimulator.h"
#include "stdexcept"


#define WHITE {1,1,1}

//------------------------------
// Constructor
//------------------------------
MassSpringSystemSimulator::MassSpringSystemSimulator():m_fMass(10), m_fStiffness(40){
	this->reset();

	//initialize one massspring
	int p1 = addMassPoint({0,0,0}, {-1,0,0}, true);
	int p2 = addMassPoint({0,2,0}, {1,0,0}, false);
	addSpring(p1, p2, 1);
}

//------------------------------
// Helper Functions
//------------------------------
float MassSpringSystemSimulator::calcDistance(Vec3 p1, Vec3 p2){
	float diffX = std::abs(p1.x - p2.x);
	float diffY = std::abs(p1.y - p2.y);
	float diffZ = std::abs(p1.z - p2.z);
	return std::sqrt(std::pow(diffX, 2) + std::pow(diffY, 2) + std::pow(diffZ, 2));
}

void MassSpringSystemSimulator::computeEuler(float timeElapsed, int initialLength, float stiffness){
		
}

void MassSpringSystemSimulator::computeMidPoint(float timeElapsed, int initialLength, float stiffness){

}

//------------------------------
// UI
//------------------------------
const char* MassSpringSystemSimulator::getTestCasesStr(){
	return "Euler, Midpoint, Leap-Forg";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass *DUC){
	this->DUC = DUC;
	for(const auto &s: springs){
		TwAddSeparator(DUC->g_pTweakBar, "separator", NULL);
		TwAddVarRO(DUC->g_pTweakBar, "InitialLength", TW_TYPE_FLOAT, &s.initialLength, "");
	}
}

void MassSpringSystemSimulator::reset(){
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	if(springs.size() > 0){
		masspoints[springs[0].masspoint1].position = {0,0,0};
		masspoints[springs[0].masspoint1].velocity= {-1,0,0};
		masspoints[springs[0].masspoint2].position = {0,2,0};
		masspoints[springs[0].masspoint2].velocity= {1,0,0};
	}
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext *pd3dImmediateContext){
	for(const auto &s : springs){
		DUC->beginLine();
		DUC->drawLine(getPositionOfMassPoint(s.masspoint1), WHITE, getPositionOfMassPoint(s.masspoint2), WHITE);
		DUC->endLine();
		DUC->drawSphere(this->getPositionOfMassPoint(s.masspoint1), 0.05);
		DUC->drawSphere(this->getPositionOfMassPoint(s.masspoint2), 0.05);
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase){
	m_iTestCase = testCase;
	switch(m_iTestCase){
		case 0:
			std::cout << "Euler" <<std::endl;
			this->reset();
			break;
		case 1:
			std::cout << "Midpoint" << std::endl;
			this->reset();
			break;
		case 2:
			std::cout << "Leap-Forg" << std::endl;
			this->reset();
			break;
		default:
			break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed){
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		// diable horizontal movement
		mouseDiff.x = 0;
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		//TODO: still have bugs somehow
		masspoints[springs[0].masspoint1].position += inputWorld;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep){
	//TODO
}

void MassSpringSystemSimulator::onClick(int x, int y){
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y){
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

//------------------------------
// Setters and Getters
//------------------------------
void MassSpringSystemSimulator::setMass(float mass){m_fMass = mass;}
void MassSpringSystemSimulator::setStiffness(float stiffness){m_fDamping = stiffness;}
void MassSpringSystemSimulator::setDampingFactor(float damping){m_fDamping = damping;}

int MassSpringSystemSimulator::getNumberOfMassPoints(){return masspoints.size();}
int MassSpringSystemSimulator::getNumberOfSprings(){return springs.size();}
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index){return masspoints.at(index).position;}
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index){return masspoints.at(index).velocity;}

//-----------------------------
// Specific Functions
//-----------------------------
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed){
	struct MassPoint masspoint;
	masspoint.position = position;
	masspoint.velocity = Velocity;
	masspoint.isFixed = isFixed;
	masspoints.emplace_back(masspoint);
	return masspoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength){
	if(masspoint1 >= masspoints.size() || masspoint2 >= masspoints.size()){
		throw std::invalid_argument("masspoints index bigger than the size of masspoints");
	}
	struct Spring spring;
	spring.masspoint1 = masspoint1;
	spring.masspoint2 = masspoint2;
	spring.initialLength = initialLength;
	springs.emplace_back(spring);
}


void MassSpringSystemSimulator::applyExternalForce(Vec3 force){
	//TODO
}
