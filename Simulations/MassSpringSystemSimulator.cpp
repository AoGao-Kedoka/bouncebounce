#include "MassSpringSystemSimulator.h"
#include "stdexcept"


#define WHITE {1,1,1}

//------------------------------
// Constructor
//------------------------------
MassSpringSystemSimulator::MassSpringSystemSimulator():m_fMass(10), m_fStiffness(40){
	this->reset();

	//initialize one massspring
	int p1 = addMassPoint({0,0,0}, {-1,0,0}, false);
	int p2 = addMassPoint({0,2,0}, {1,0,0}, false);
	addSpring(p1, p2, 1);
}

//------------------------------
// UI
//------------------------------
const char* MassSpringSystemSimulator::getTestCasesStr(){
	return "Euler, Midpoint, Leap-Forg";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass *DUC){
	this->DUC = DUC;
	//TODO: init UI to generate springs and masspoints
}

void MassSpringSystemSimulator::reset(){
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext *pd3dImmediateContext){
	for(const auto &s : springs){
		DUC->beginLine();
		DUC->drawLine(getPositionOfMassPoint(s.masspoint1), WHITE, getPositionOfMassPoint(s.masspoint2), WHITE);
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase){
	m_iTestCase = testCase;
	switch(m_iTestCase){
		case 0:
			std::cout << "Euler" <<std::endl;
			break;
		case 1:
			std::cout << "Midpoint" << std::endl;
			break;
		case 2:
			std::cout << "Leap-Forg" << std::endl;
			break;
		default:
			break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed){
	//TODO
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
// Setter and Getter
//------------------------------
void MassSpringSystemSimulator::setMass(float mass){m_fMass = mass;}
void MassSpringSystemSimulator::setStiffness(float stiffness){m_fDamping = stiffness;}
void MassSpringSystemSimulator::setDampingFactor(float damping){m_fDamping = damping;}

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

int MassSpringSystemSimulator::getNumberOfMassPoints(){return masspoints.size();}
int MassSpringSystemSimulator::getNumberOfSprings(){return springs.size();}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index){return masspoints.at(index).position;}
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index){return masspoints.at(index).velocity;}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force){
	//TODO
}
