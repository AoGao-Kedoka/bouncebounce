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

void computeMassDensity(Particle*  p)
{
	Vec3 r_ij = p->pos - p->tmp_pos;

	float  r = p->pos[0] - p->tmp_pos[0];
	float r2 = std::pow(r, 2);

	if (r2 <= 0.002) {
		p->rho = 0.002 * 315.f / (64.f * 3.1415 * std::pow(0.046, 9)) * std::pow(0.001 - r2, 3.f);
	}
}

void computePressure(Particle* p)
{
	p->p = 3.f * (p->rho - 1000.0f);
}

void computeForce(Particle* p)
{
	Vec3 f_preffusre(0, 0, 0);
	Vec3 f_viscosity(0, 0, 0);

	Vec3 r_ijv = p->pos - p->tmp_pos;
	float r_ij = r_ijv[0];
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

void SPHSimulator::simulateTimestep(float timeStep)
{
	for (size_t i = 0; i < _width; ++i) {
		for (size_t j = 0; j < _height; ++j) {
			for (size_t k = 0; k < _length; ++k) {
				computeMassDensity(&particles[i][j][k]);
				computePressure(&particles[i][j][k]);
			}
		}
	}

	for (size_t i = 0; i < _width; ++i) {
		for (size_t j = 0; j < _height; ++j) {
			for (size_t k = 0; k < _length; ++k) {
				computeForce(&particles[i][j][k]);
			}
		}
	}

	for (size_t i = 0; i < _width; ++i) {
		for (size_t j = 0; j < _height; ++j) {
			for (size_t k = 0; k < _length; ++k) {
				computeLocation(&particles[i][j][k], timeStep);
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