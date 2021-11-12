#include "MassSpringSystemSimulator.h"
#include "stdexcept"


#define WHITE {1,1,1}

//------------------------------
// Constructor
//------------------------------
MassSpringSystemSimulator::MassSpringSystemSimulator():m_fMass(10), m_fStiffness(40), m_frestLength(1), m_bGravityToogle(false), m_iIntegrator(0){
	this->reset();

	// initialize gravity
	m_vGravity.x = m_vGravity.z = 0;
	m_vGravity.y = -10;
}

//------------------------------
// Helper Functions
//------------------------------
float MassSpringSystemSimulator::calcDistance(Vec3 p1, Vec3 p2, Vec3 *direction){
	float diffX = p1.x - p2.x;
	float diffY = p1.y - p2.y;
	float diffZ = p1.z - p2.z;
	direction->x = diffX;
	direction->y = diffY;
	direction->z = diffZ;
	return std::sqrt(std::pow(diffX, 2) + std::pow(diffY, 2) + std::pow(diffZ, 2));
}

std::pair<Vec3, Vec3> MassSpringSystemSimulator::calcAcc(Vec3 position1, Vec3 position2){
		Vec3 direction;
		auto distance = this->calcDistance(position1, position2, &direction);
		Vec3 force = m_fStiffness * (distance - m_frestLength) * (direction/distance);
		
		Vec3 a1 = - force / m_fMass;
		Vec3 a2 = force / m_fMass;
		if(m_bGravityToogle){
			a1 += m_vGravity;
			a2 += m_vGravity;	
		}
		return std::make_pair(a1, a2);
}

void MassSpringSystemSimulator::buildSprings(int number){
	if (number == 0)
		return;
	if(number == 1){
		Vec3 position1 = {0,0,0};
		Vec3 position2 = {0,2,0};
		Vec3 velocity1 = {-1,0,0};
		Vec3 velocity2 = {1,0,0};
		int p1 = addMassPoint(position1, velocity1, false);
		int p2 = addMassPoint(position2, velocity2, false);
		addSpring(p1, p2, m_frestLength);
	} else {
		//TODO
	}
}

void MassSpringSystemSimulator::computeEuler(float timeStep){
	for(const auto &s : springs){
		auto *mp1 = &masspoints[s.masspoint1];
		auto *mp2 = &masspoints[s.masspoint2];

		// update position
		mp1->position += timeStep * mp1->velocity;
		mp2->position += timeStep * mp2->velocity;
		std::cout<<"Euler--"<<"Masspoint1's position: "<< mp1->position << std::endl; 
		std::cout<<"Euler--"<<"Masspoint2's position: "<< mp2->position << std::endl; 

		// update velocity	
		auto acc = this->calcAcc(mp1->position, mp2->position);
		mp1->velocity += timeStep * acc.first;
		mp2->velocity += timeStep * acc.second;
		std::cout<< "Euler--"<<"Masspoint1's velocity: "<< mp1->velocity << std::endl;
		std::cout<< "Euler--"<<"Masspoint2's velocity: "<< mp1->velocity << std::endl;
	}
}

void MassSpringSystemSimulator::computeMidPoint(float timeStep){
	for(const auto &s : springs){
		auto *mp1 = &masspoints[s.masspoint1];
		auto *mp2 = &masspoints[s.masspoint2];

		// compute midpoint
		auto midStepPos1 = mp1->position + 1/2 * timeStep * mp1->velocity;
		auto midStepPos2 = mp2->position + 1/2 * timeStep * mp2->velocity;

		auto acc = this->calcAcc(mp1->position, mp2->position);

		auto midStepVel1 = mp1->velocity + 1/2 * timeStep * acc.first;
		auto midStepVel2 = mp2->velocity + 1/2 * timeStep * acc.second;
		acc = this->calcAcc(midStepPos1, midStepPos2);
		
		// compute full step
		mp1->position += timeStep * midStepVel1;
		mp2->position += timeStep * midStepVel2;
		std::cout<<"Midpoint--"<<"Masspoint1's position: "<< mp1->position << std::endl; 
		std::cout<<"Midpoint--"<<"Masspoint2's position: "<< mp2->position << std::endl; 
		mp1->velocity += timeStep * acc.first;
		mp2->velocity += timeStep * acc.second;
		std::cout<<"Midpoint--"<<"Masspoint1's velocity: " << mp1->velocity << std::endl;
		std::cout<<"Midpoint--"<<"Masspoint2's velocity: " << mp1->velocity << std::endl;
	}
}

void MassSpringSystemSimulator::computeLeapFrog(float timeStep){
	// TODO: should do last
}

//------------------------------
// UI
//------------------------------
const char* MassSpringSystemSimulator::getTestCasesStr(){
	return "DemoForTest, Demo1, Demo2, Demo3, Demo4";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass *DUC){
	this->DUC = DUC;
	TwAddSeparator(DUC->g_pTweakBar, "SeperatorUp", "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &m_bGravityToogle, "");
	if(m_iTestCase == 1){
		TwAddButton(DUC->g_pTweakBar, "time step help", NULL, NULL, "label='please ignore time step above'");
		TwAddButton(DUC->g_pTweakBar, "time step0", NULL, NULL, "label='new time step = 0.1'");
	}
	else if(m_iTestCase == 2 || m_iTestCase == 3){
		TwAddButton(DUC->g_pTweakBar, "time step help", NULL, NULL, "label='please ignore time step above'");
		TwAddButton(DUC->g_pTweakBar, "time step", NULL, NULL, "label='new time step = 0.005'");
		TwAddVarRO(DUC->g_pTweakBar, "Integrate", TW_TYPE_INT8, &m_iIntegrator, "");
		TwRemoveVar(DUC->g_pTweakBar, "time step0");
	} else if(m_iTestCase == 4){
		TwRemoveVar(DUC->g_pTweakBar, "time step0");
		TwRemoveVar(DUC->g_pTweakBar, "time step");
		TwRemoveVar(DUC->g_pTweakBar, "time step help");
		TwAddVarRW(DUC->g_pTweakBar, "Integrate1", TW_TYPE_INT8, &m_iIntegrator, "");
	}
}

void MassSpringSystemSimulator::reset(){
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	int currentNum = springs.size();
	springs.clear();
	masspoints.clear();
	this->buildSprings(currentNum);
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
			std::cout << "Demo for test usage" << std::endl;
			break;	
		case 1:
			this->buildSprings(1);
			std::cout << "Demo1" <<std::endl;
			break;
		case 2:
			this->reset();
			std::cout << "Demo2" << std::endl;
			break;
		case 3:
			this->reset();
			std::cout << "Demo3" << std::endl;
			break;
		case 4:
			this->reset();
			std::cout << "Demo4 with Euler" << std::endl;
			this->buildSprings(10);
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
		//TODO: Update position
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep){
	switch(m_iTestCase){
		case 0:
			if(m_iIntegrator == 0)
				this->computeEuler(timeStep);
			else if(m_iIntegrator == 1)
				this->computeMidPoint(timeStep);
			break;
		case 1:
			// only compute 1 time
			if(masspoints[springs[0].masspoint1].position.x == 0){
				this->computeEuler(0.1);	
				this->reset();
				if(masspoints[springs[0].masspoint1].position.x == 0)
					this->computeMidPoint(0.1);
			}
			break;
		case 2:
			this->computeEuler(0.005);
			break;
		case 3:
			this->computeMidPoint(0.005);
			break;
		case 4:
			if(m_iIntegrator == 0)
				this->computeEuler(timeStep);
			else if (m_iIntegrator == 1)
				this->computeMidPoint(timeStep);
			break;
		default:
			if(m_iIntegrator == 0)
				this->computeEuler(timeStep);
			else if(m_iIntegrator == 2)
				this->computeMidPoint(timeStep);
			break;
	}
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
