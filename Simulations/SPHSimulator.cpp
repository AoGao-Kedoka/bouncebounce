#include "SPHSimulator.h"

#define GRAVITY Vec3(0, -9.8,0);

SPHSimulator::SPHSimulator(int width, int height, int length) :
	_width(width), _length(length), _height(height)
{
	buildParticles(width, height, length);
}

SPHSimulator::~SPHSimulator() {
	cleanupParticles();
}

void SPHSimulator::cleanupParticles() {
	particles.clear();
}

void SPHSimulator::buildParticles(int width, int length, int height) {
	for (size_t i = 0; i < width; ++i) {
		for (size_t j = 0; j < height; ++j) {
			for (size_t k = 0; k < length; ++k) {
				Particle p{
						Vec3((float)i * _distance_between,(float)j * (_distance_between),(float)k * _distance_between), //position
						1 / M, // mass 
						Vec3(0,0.1, 0.3) //color
				};
				particles.push_back(p);
			}
		}
	}
	this->reset();
}
const char* SPHSimulator::getTestCasesStr()
{
	return "WATER (1000),WATER (2197),\
			MUCUS (1000),MUCUS (2197),\
			GAS (1000),GAS (2197)";
}

void SPHSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void SPHSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void SPHSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (auto &p : particles) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, p.color);
		DUC->drawSphere(p.pos -
			Vec3(
				(_width * _distance_between) / 2, 
				(_length * _distance_between) / 2, 
				(_height * _distance_between) / 2), 
			Vec3(0.019f, 0.019f, 0.019f)
		);
	}
}

void SPHSimulator::setConstants(int state) {
	if (state == STATE_WATER) {
		H = 0.0457f;
		M = 0.02f;
		MU = 3.5f;
		REST_DENS = 998.29f;
		B = 0;
		CURR_STATE = STATE_WATER;
	}
	else if (state == STATE_MUCUS) {
		H = 0.0726f;
		M = 0.04f;
		MU = 36.f;
		REST_DENS = 1000.0f;
		B = 0;
		CURR_STATE = STATE_MUCUS;
	}
	else if (state == STATE_GAS) {
		H = 0.06f;
		REST_DENS = 0.59f;
		M = 0.00005f;
		MU = 0.01f;
		B = 5;
		CURR_STATE = STATE_GAS;
	}
	H2 = H * H;
	H6 = H * H * H * H * H * H;
	H9 = H * H * H * H * H * H * H * H * H;
	POLY6 = 315.f / (64.f * PI * H9);
	POLY6_GRAD_PRESS = -45.f / (PI * H6);
	POLY6_GRAD_VISC = 45.f / (PI * H6);
}

void SPHSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase) {
	case 0:
		setConstants(STATE_WATER);
		cleanupParticles();
		buildParticles(10, 10, 10);
		break;
	case 1:
		setConstants(STATE_WATER);
		cleanupParticles();
		buildParticles(13, 13, 13);
		break;
	case 2:
		setConstants(STATE_MUCUS);
		cleanupParticles();
		buildParticles(10, 10, 10);
		break;
	case 3:
		setConstants(STATE_MUCUS);
		cleanupParticles();
		buildParticles(13, 13, 13);
		break;
	case 4:
		setConstants(STATE_GAS);
		cleanupParticles();
		buildParticles(10, 10, 10);
		break;
	case 5:
		setConstants(STATE_GAS);
		cleanupParticles();
		buildParticles(13, 13, 13);
		break;
	}
}

void SPHSimulator::externalForcesCalculations(float timeElapsed)
{
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
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		for (Particle &p : particles) {
			p.pos += inputWorld;
		}
	}
}

// Compute density and pressure of particle p
void SPHSimulator::computeDensityAndPressure(Particle* p)
{
	// Compute Density
	p->rho = 0.f;
	// p_j loops over all other particles
	for (auto &p_j : particles) {
		Vec3 r_ij = p_j.pos - p->pos;
		// Euclidean distance
		float r = norm(p_j.pos - p->pos);
		float r2 = r * r;

		// r is inside the kernel radius (0 otherwise)
		if (r2 <= H2)
			p->rho += M * POLY6 * std::pow(H2 - r2, 3.f);
	}

	// Compute Pressure
	p->p = GAS_CONST * (p->rho - REST_DENS);	
}

void SPHSimulator::computeForces(Particle* p)
{
	Vec3 f_pressure(0., 0., 0.);
	Vec3 f_viscosity(0., 0., 0.);
	for (auto &p_j : particles) {
		// TODO do this better
		// i == j 
		if (p_j.pos.x == p->pos.x &&
			p_j.pos.y == p->pos.y &&
			p_j.pos.z == p->pos.z) continue;

		Vec3 r_ijv = p->pos - p_j.pos;
		float r_ij = norm(r_ijv);

		if (r_ij <= H) {
			// compute pressure force contribution

			f_pressure += -M * (p->p + p_j.p) / (2 * p_j.rho) *
				POLY6_GRAD_PRESS * r_ijv / r_ij * std::pow(H - r_ij, 2.0f);

			f_viscosity += (p_j.v - p->v) * M / p_j.rho *
				POLY6_GRAD_VISC * (H - r_ij);
		}
	}

	f_viscosity *= MU;

	Vec3 f_gravity = p->rho * GRAVITY;

	// B == 0 when not gas
	if(B > 0) f_gravity = B * (p->rho - REST_DENS) * GRAVITY;

	p->f = f_pressure + f_viscosity + f_gravity;
}

void SPHSimulator::computeLocation(Particle* p, float timeStep)
{
	// compute velocity using forward euler
	//Vec3 acc = timeStep * p->f / p->rho; // + GRAVITY;
	p->v += timeStep * p->f / p->rho;
	p->pos += timeStep * p->v;
	
	// boundary walls
	if (p->pos.x <= MIN_X) {
		p->v.x *= -0.5;
		p->pos.x = MIN_X + EPS;
	}
	if (p->pos.x >= MAX_X) {
		p->v.x *= -0.5;
		p->pos.x = MAX_X - EPS;
	}
	if (p->pos.z <= MIN_X) {
		p->v.z *= -0.5;
		p->pos.z = MIN_X + EPS;
	}
	if (p->pos.z >= MAX_X) {
		p->v.z *= -0.5;
		p->pos.z = MAX_X - EPS;
	}
	// ground
	if (p->pos.y <= MIN_Y) {
		p->v.y *= -0.5;
		p->pos.y = MIN_Y + EPS;
	}
	// sky, only for gas
	if (B > 0.f && p->pos.y >= MAX_Y) {
		p->v.y *= -0.5;
		p->pos.y = MAX_Y - EPS;
	}

	// dampen velocity
	p->v *= 0.98;
}

void SPHSimulator::simulateTimestep(float timeStep)
{
	for(Particle &p: particles)
		computeDensityAndPressure(&p);

	for (Particle &p : particles)
		computeForces(&p);

	for (Particle &p : particles)
		computeLocation(&p, timeStep);

	float maxP = particles[0].p;
	float minP = particles[0].p;
	for (Particle &p : particles)
		if (p.p > maxP) maxP = p.p;
		else if (p.p < minP) minP = p.p;

	// color particles based on density
	for (Particle &p : particles) {
		double r = (p.p + minP) / abs((maxP - minP));
		if(CURR_STATE == STATE_WATER)
			p.color = Vec3(r, 0.1, 0.3) * 0.2 + p.color * 0.8;
		else if (CURR_STATE == STATE_MUCUS)
			p.color = Vec3(r, 0.3, 0.1) * 0.2 + p.color * 0.8;
		else 
			p.color = Vec3(1-r/2, 1-r/2, 1-r/2) * 0.2 + p.color * 0.8;
	}
}

void SPHSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void SPHSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}