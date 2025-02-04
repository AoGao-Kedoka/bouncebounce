#include "MassSpringSystemSimulator.h"
#include "stdexcept"

#define COLOR {0.3,0.3,0.6}

//------------------------------
// Constructor and Destructor
//------------------------------
MassSpringSystemSimulator::MassSpringSystemSimulator() :m_fMass(10), m_fStiffness(40), m_frestLength(1), m_bGravityToogle(false), m_iIntegrator(0) {
	this->reset();

	// initialize gravity
	m_vGravity = Vec3(0, -10, 0);
}

MassSpringSystemSimulator::~MassSpringSystemSimulator() {
	springs.clear();
	masspoints.clear();
}

//------------------------------
// Helper Functions
//------------------------------
float MassSpringSystemSimulator::calcDistance(Vec3 p1, Vec3 p2, Vec3* direction) {
	float diffX = p1.x - p2.x;
	float diffY = p1.y - p2.y;
	float diffZ = p1.z - p2.z;
	direction->x = diffX;
	direction->y = diffY;
	direction->z = diffZ;
	return std::sqrt(std::pow(diffX, 2) + std::pow(diffY, 2) + std::pow(diffZ, 2));
}

std::pair<Vec3, Vec3> MassSpringSystemSimulator::calcAcc(Vec3 position1, Vec3 position2) {
	Vec3 direction;
	auto distance = this->calcDistance(position1, position2, &direction);
	Vec3 force = -m_fStiffness * (distance - m_frestLength) * (direction / distance);

	Vec3 a1 = force / m_fMass;
	// because same mass and same forces => performance improvement
	Vec3 a2 = -a1;
	if (m_bGravityToogle) {
		a1 += m_vGravity;
		a2 += m_vGravity;
	}
	return std::make_pair(a1, a2);
}

void MassSpringSystemSimulator::buildSprings(int number) {
	springs.clear();
	masspoints.clear();
	if (number == 0)
		return;
	if (number == 1) {
		Vec3 position1 = { 0,0,0 };
		Vec3 position2 = { 0,2,0 };
		Vec3 velocity1 = { -1,0,0 };
		Vec3 velocity2 = { 1,0,0 };
		int p1 = addMassPoint(position1, velocity1, false);
		int p2 = addMassPoint(position2, velocity2, false);
		addSpring(p1, p2, m_frestLength);
	}
	else {
		for (int i = 0; i < number; i++)
			for (int j = 0; j < number; j++)
				addMassPoint(
					Vec3(
						(float)i / 2 - 0.4,
						(float)j / 2 - 0.4,
						0.f
					),
					Vec3(0, 0, 0),
					false);

		for (int i = 0; i < masspoints.size() - number; i++) {
			if (!(i % number == 1))
				addSpring(i, i + 1, m_frestLength);
			addSpring(i, i + number, m_frestLength);
		}
	}
}

void MassSpringSystemSimulator::computeEuler(float timeStep) {
	for (const auto& s : springs) {
		auto* mp1 = &masspoints[s.masspoint1];
		auto* mp2 = &masspoints[s.masspoint2];

		// update position
		mp1->position += timeStep * mp1->velocity;
		mp2->position += timeStep * mp2->velocity;

		// update velocity
		auto acc = this->calcAcc(mp1->position, mp2->position);
		mp1->velocity += timeStep * acc.first;
		mp2->velocity += timeStep * acc.second;
		if (mp1->position.y <= -1)
			mp1->velocity = -mp1->velocity;
		if (mp2->position.y <= -1)
			mp2->velocity = -mp2->velocity;
	}
}

void MassSpringSystemSimulator::computeMidPoint(float timeStep) {
	for (const auto& s : springs) {
		auto* mp1 = &masspoints[s.masspoint1];
		auto* mp2 = &masspoints[s.masspoint2];

		// compute midpoint
		auto midStepPos1 = mp1->position + 1 / 2 * timeStep * mp1->velocity;
		auto midStepPos2 = mp2->position + 1 / 2 * timeStep * mp2->velocity;

		auto acc = this->calcAcc(mp1->position, mp2->position);

		auto midStepVel1 = mp1->velocity + 1 / 2 * timeStep * acc.first;
		auto midStepVel2 = mp2->velocity + 1 / 2 * timeStep * acc.second;
		acc = this->calcAcc(midStepPos1, midStepPos2);

		// compute full step
		mp1->position += timeStep * midStepVel1;
		mp2->position += timeStep * midStepVel2;
		mp1->velocity += timeStep * acc.first;
		mp2->velocity += timeStep * acc.second;

		// bounce bounce!!!
		if (mp1->position.y <= -1)
			mp1->velocity = -mp1->velocity;
		if (mp2->position.y <= -1)
			mp2->velocity = -mp2->velocity;
	}
}

void MassSpringSystemSimulator::computeLeapFrog(float timeStep) {
	for (const auto& s : springs) {
		auto* mp1 = &masspoints[s.masspoint1];
		auto* mp2 = &masspoints[s.masspoint2];

		// update velocity first
		auto acc = this->calcAcc(mp1->position, mp2->position);
		mp1->velocity += timeStep * acc.first;
		mp2->velocity += timeStep * acc.second;

		// update position then
		mp1->position += timeStep * mp1->velocity;
		mp2->position += timeStep * mp2->velocity;

		if (mp1->position.y <= -1)
			mp1->velocity = -mp1->velocity;
		if (mp2->position.y <= -1)
			mp2->velocity = -mp2->velocity;
	}
}

void MassSpringSystemSimulator::writeState() {
	for (int i = 0; i < masspoints.size(); i++) {
		std::cout << "Masspoint " << i << "'s position: " << masspoints[i].position << std::endl;
		std::cout << "Masspoint " << i << "'s velocity: " << masspoints[i].velocity << std::endl;
	}
}

//------------------------------
// UI
//------------------------------
const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo 0: For testing, \
            Demo 1: A simple one-step test, \
            Demo 2: A simple Euler simulation, \
            Demo 3: A simple Midpoint simulation, \
            Demo 4: A complex simulation -- comparing stability of Euler and Midpoint,\
			Demo 5: A simple Leap Frog simulation";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwRemoveVar(DUC->g_pTweakBar, "Timestep");
	TwAddVarRO(DUC->g_pTweakBar, "Timestep", TW_TYPE_FLOAT, &m_ftimeStep_Cur, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &m_bGravityToogle, "");
	if (m_iTestCase == 2 || m_iTestCase == 3 || m_iTestCase == 5) {
		TwAddButton(DUC->g_pTweakBar, "Info1", NULL, NULL, "label='0=Euler'");
		TwAddButton(DUC->g_pTweakBar, "Info2", NULL, NULL, "label='1=Leap-Frog'");
		TwAddButton(DUC->g_pTweakBar, "Info3", NULL, NULL, "label='2=Midpoint'");
		TwAddVarRO(DUC->g_pTweakBar, "Integrate", TW_TYPE_INT8, &m_iIntegrator, "");
	}
	else if (m_iTestCase == 4) {
		TwAddButton(DUC->g_pTweakBar, "Info1", NULL, NULL, "label='0=Euler'");
		TwAddButton(DUC->g_pTweakBar, "Info2", NULL, NULL, "label='1=Leap-Frog'");
		TwAddButton(DUC->g_pTweakBar, "Info3", NULL, NULL, "label='2=Midpoint'");
		TwRemoveVar(DUC->g_pTweakBar, "Integrate");
		TwAddVarRW(DUC->g_pTweakBar, "Integrate", TW_TYPE_INT8, &m_iIntegrator, "min=0, max=2");
		TwRemoveVar(DUC->g_pTweakBar, "Timestep");
		TwAddVarRW(DUC->g_pTweakBar, "Timestep", TW_TYPE_FLOAT, &m_ftimeStep_Cur, "min=0,step = 0.001");
	}
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	int currentNum = springs.size();
	springs.clear();
	masspoints.clear();
	this->buildSprings(currentNum);
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	for (const auto& s : springs) {
		DUC->beginLine();
		DUC->drawLine(getPositionOfMassPoint(s.masspoint1), COLOR, getPositionOfMassPoint(s.masspoint2), COLOR);
		DUC->endLine();
		DUC->drawSphere(this->getPositionOfMassPoint(s.masspoint1), 0.05);
		DUC->drawSphere(this->getPositionOfMassPoint(s.masspoint2), 0.05);
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	switch (m_iTestCase) {
	case 0:
		std::cout << "Demo for test usage" << std::endl;
		break;
	case 1:
		this->buildSprings(1);
		std::cout << "Demo 1: A simple one-step test" << std::endl;
		break;
	case 2:
		this->buildSprings(1);
		m_iIntegrator = EULER;
		std::cout << "Demo 2: A simple Euler simulation" << std::endl;
		break;
	case 3:
		m_iIntegrator = MIDPOINT;
		this->buildSprings(1);
		std::cout << "Demo 3: A simple Midpoint simulation" << std::endl;
		break;
	case 4:
		m_iIntegrator = EULER;
		this->buildSprings(10);
		std::cout << "Demo 4: A complex simulation, comparing stability of Euler and Midpoint" << std::endl;
		break;
	case 5:
		m_iIntegrator = LEAPFROG;
		this->buildSprings(1);
		std::cout << "Demo 5: A simple Leap-Frog simulation" << std::endl;
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.00001f;
		inputWorld = inputWorld * inputScale;
		for (int i = 0; i < masspoints.size(); i++)
			masspoints[i].position += inputWorld;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	switch (m_iTestCase) {
	case 0:
		if (m_iIntegrator == EULER)
			this->computeEuler(timeStep);
		else if (m_iIntegrator == LEAPFROG)
			this->computeLeapFrog(timeStep);
		else if (m_iIntegrator == MIDPOINT)
			this->computeMidPoint(timeStep);
		break;
	case 1:
		// do only 1 iteration
		m_ftimeStep_Cur = 0.1;
		if (masspoints[springs[0].masspoint1].position.x == 0) {
			this->computeEuler(m_ftimeStep_Cur);
			std::cout << "-- Calculated with Euler method" << std::endl;
			this->writeState();
			this->reset();
			if (masspoints[springs[0].masspoint1].position.x == 0) {
				this->computeMidPoint(m_ftimeStep_Cur);
				std::cout << "-- Calculated with Midpoint method" << std::endl;
				this->writeState();
			}
		}
		break;
	case 2:
		m_ftimeStep_Cur = 0.005;
		this->computeEuler(m_ftimeStep_Cur);
		break;
	case 3:
		m_ftimeStep_Cur = 0.005;
		this->computeMidPoint(m_ftimeStep_Cur);
		break;
	case 4:
		m_ftimeStep_Cur = timeStep;
		if (m_iIntegrator == 0)
			this->computeEuler(m_ftimeStep_Cur);
		else if (m_iIntegrator == 1)
			this->computeLeapFrog(m_ftimeStep_Cur);
		else if (m_iIntegrator == 2)
			this->computeMidPoint(m_ftimeStep_Cur);
		break;
	case 5:
		m_ftimeStep_Cur = 0.005;
		this->computeLeapFrog(m_ftimeStep_Cur);
		break;
	default:
		if (m_iIntegrator == 0)
			this->computeEuler(timeStep);
		else if (m_iIntegrator == 1)
			this->computeLeapFrog(timeStep);
		else if (m_iIntegrator == 2)
			this->computeMidPoint(timeStep);
		break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

//------------------------------
// Setters and Getters
//------------------------------
void MassSpringSystemSimulator::setMass(float mass) { m_fMass = mass; }
void MassSpringSystemSimulator::setStiffness(float stiffness) { m_fDamping = stiffness; }
void MassSpringSystemSimulator::setDampingFactor(float damping) { m_fDamping = damping; }

int MassSpringSystemSimulator::getNumberOfMassPoints() { return masspoints.size(); }
int MassSpringSystemSimulator::getNumberOfSprings() { return springs.size(); }
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) { return masspoints.at(index).position; }
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) { return masspoints.at(index).velocity; }

//-----------------------------
// Specific Functions
//-----------------------------
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	struct MassPoint masspoint;
	masspoint.position = position;
	masspoint.velocity = Velocity;
	masspoint.isFixed = isFixed;
	masspoints.push_back(masspoint);
	return masspoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	if (masspoint1 >= masspoints.size() || masspoint2 >= masspoints.size()) {
		throw std::invalid_argument("masspoints index bigger than the size of masspoints");
	}
	struct Spring spring;
	spring.masspoint1 = masspoint1;
	spring.masspoint2 = masspoint2;
	spring.initialLength = initialLength;
	springs.push_back(spring);
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	// makes no difference
}