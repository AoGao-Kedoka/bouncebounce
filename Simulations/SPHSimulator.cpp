#include "SPHSimulator.h"

#define GRAVITY Vec3(0, -0.98,0);
typedef std::vector < std::vector<std::vector<Particle>>> field;
SPHSimulator::SPHSimulator(int width, int height, int length) :
	_width(width), _length(length), _height(height)
{
	for (size_t i = 0; i < width; ++i) {
		std::vector<std::vector<Particle>> jv;
		for (size_t j = 0; j < height; ++j) {
			std::vector<Particle> kv;
			for (size_t k = 0; k < length; ++k) {
				Particle p{ Vec3((float)i * _distance_between,(float)j * _distance_between,(float)k * _distance_between), 0, Vec3(0,0.1, 0.3) };
				kv.push_back(p);
			}
			jv.push_back(kv);
		}
		particles.push_back(jv);
	}
	this->reset();
}

const char* SPHSimulator::getTestCasesStr()
{
	return "SPH";
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
	for (size_t i = 0; i < _width; ++i) {
		for (size_t j = 0; j < _height; ++j) {
			for (size_t k = 0; k < _length; ++k) {
				DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, particles[i][j][k].color);
				DUC->drawSphere(particles[i][j][k].pos -
					Vec3((_width * _distance_between) / 2, (_length * _distance_between) / 2, (_height * _distance_between) / 2), Vec3(0.02f, 0.02f, 0.02f));
			}
		}
	}
}

void SPHSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
}

void SPHSimulator::externalForcesCalculations(float timeElapsed)
{
}

void computeMassDensity(Particle p)
{
}

void computePressure(Particle p)
{
}

void computeForce(Particle p)
{
}

void computeLocation(Particle* p, float timeStep)
{
	// compute velocity using Leap Frog
	Vec3 acc = timeStep * p->f / p->rho + GRAVITY;
	p->v += timeStep * acc;
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
	
	// groud
	if (p->pos.y <= MIN_Y) {
		p->v.y *= -0.5;
		p->pos.y = MIN_Y + EPS;
	}

}

void detectCollision(field particles, Particle* p)
{
	
}

void SPHSimulator::simulateTimestep(float timeStep)
{
	for (size_t i = 0; i < _width; ++i) {
		for (size_t j = 0; j < _height; ++j) {
			for (size_t k = 0; k < _length; ++k) {
				computeMassDensity(particles[i][j][k]);
				computePressure(particles[i][j][k]);
			}
		}
	}

	for (size_t i = 0; i < _width; ++i) {
		for (size_t j = 0; j < _height; ++j) {
			for (size_t k = 0; k < _length; ++k) {
				computeForce(particles[i][j][k]);
			}
		}
	}

	for (size_t i = 0; i < _width; ++i) {
		for (size_t j = 0; j < _height; ++j) {
			for (size_t k = 0; k < _length; ++k) {
				computeLocation(&particles[i][j][k], timeStep);
				detectCollision(particles, &particles[i][j][k]);
			}
		}
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