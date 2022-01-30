#include "SPHSimulator.h"
#include <set>
#define GRAVITY Vec3(0, -9.8,0);

SPHSimulator::SPHSimulator(int width, int height, int length) :
	_width(width), _length(length), _height(height)
{
	for (size_t i = 0; i < width; ++i) {
		for (size_t j = 0; j < height; ++j) {
			for (size_t k = 0; k < length; ++k) {
				particles.push_back(
					new Particle{
						Vec3((float)i * _distance_between,(float)j * _distance_between,(float)k * _distance_between), //position
						1 / M, // mass 
						Vec3(0,0.1, 0.3) //color
					}
				);
			}
		}
	}
	this->reset();
}

SPHSimulator::~SPHSimulator() {
	for (auto p : particles)
		delete p;
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
	for (auto p : particles) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, p->color);
		DUC->drawSphere(p->pos -
			Vec3(
				(_width * _distance_between) / 2, 
				(_length * _distance_between) / 2, 
				(_height * _distance_between) / 2), 
			Vec3(0.02f, 0.02f, 0.02f)
		);
	}
}

void SPHSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
}

void SPHSimulator::externalForcesCalculations(float timeElapsed)
{
}

// Compute density and pressure of particle p
void SPHSimulator::computeDensityAndPressure(Particle* p)
{
	// Compute Density
	p->rho = 0.f;
	// p_j loops over all other particles

	//for (auto &p_j : this->spatialHash->collisions(p)) {
	for (auto &p_j : particles) {

		Vec3 r_ij = p_j->pos - p->pos;
		// Euclidean distance
		float r = norm(p_j->pos - p->pos);
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

	for (auto p_j  : this->spatialHash->collisions(p)) {
		// TODO do this better
		// i == j 
		if (p_j->pos.x == p->pos.x &&
			p_j->pos.y == p->pos.y &&
			p_j->pos.z == p->pos.z) continue;

		Vec3 r_ijv = p->pos - p_j->pos;
		float r_ij = norm(r_ijv);

		if (r_ij <= H) {
			// compute pressure force contribution

			f_pressure += -M * (p->p + p_j->p) / (2 * p_j->rho) *
				POLY6_GRAD_PRESS * r_ijv / r_ij * std::pow(H - r_ij, 2.0f);

			f_viscosity += (p_j->v - p->v) * M / p_j->rho *
				POLY6_GRAD_VISC * (H - r_ij);
		}
	}

	f_viscosity *= MU;

	Vec3 f_gravity = p->rho * GRAVITY;

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

	// dampen velocity
	p->v *= 0.98;
}

void SPHSimulator::simulateTimestep(float timeStep)
{
	this->spatialHash = new SpatialHash{ this->particles, this->H*100};

	for(Particle* p: particles)
		computeDensityAndPressure(p);

	for (Particle* p : particles)
		computeForces(p);

	for (Particle* p : particles)
		computeLocation(p, timeStep);

	float maxP = particles[0]->p;
	float minP = particles[0]->p;
	for (Particle* p : particles)
		if (p->p > maxP) maxP = p->p;
		else if (p->p < minP) minP = p->p;

	for (Particle* p : particles) {
		p->color = Vec3((p->p + minP)/abs((maxP-minP)), 0.1, 0.3) ;
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



std::ostream& operator<< (std::ostream& os, const Particle& p) {
	return	os << '(' << p.pos.x << ','
		<< p.pos.y << ','
		<< p.pos.z << ')';
}


ostream& operator<<(ostream& os, const std::vector<Particle*>& particles) {
	os << "VECTOR:" << '[';

	auto iterator = particles.begin();
	for (; iterator != particles.end() && std::next(iterator) != particles.end(); iterator++) {
		os << **iterator << ", ";
	}
	if (iterator != particles.end()) {
		os << **iterator;
	}
	os << ']';

	return os;
}

ostream& operator<<(ostream& os, const std::list<Particle*>& particles) {
	os << "LIST:" << '[';

	auto iterator = particles.begin();
	for (; iterator != particles.end() && std::next(iterator) != particles.end(); iterator++) {
		os << **iterator << ", ";
	}
	if (iterator != particles.end()) {
		os << **iterator;
	}
	os << ']';

	return os;
}

ostream& operator<<(ostream& os, const std::vector<std::list<Particle*>>& particles) {
	os << "VECTOR:" << '[';

	auto iterator = particles.begin();
	for (; iterator != particles.end() && std::next(iterator) != particles.end(); iterator++) {
		os << *iterator << ", ";
	}
	if (iterator != particles.end()) {
		os << *iterator;
	}
	os << ']';

	return os;
}

/////////////////////////////////////////////////////// SPATIAL HASH

SpatialHash::SpatialHash(const std::vector<Particle*>& particles, float h) : h{ h }, discretize{ h }, hasher{ new ParticleHasher{ h } } {
	this->hashTable.resize(this->next_prime(2 * particles.size()));

	for (Particle* particle : particles) {
		(*this)[particle->pos].push_back(particle);
	}
}

std::list<Particle*> SpatialHash::collisions(const Particle* p) {
	nVec3i bbmin = this->discretize(p->pos - Vec3(this->h, this->h, this->h));
	nVec3i bbmax = this->discretize(p->pos + Vec3(this->h, this->h, this->h));

	std::list<Particle*> collisions_list;
	std::set<Particle*> collisions;

	for (auto x = bbmin.x; x < bbmax.x; x++) {
		for (auto y = bbmin.y; y < bbmax.y; y++) {
			for (auto z = bbmin.z; z < bbmax.z; z++) {
				for (Particle* particle : (*this)[Vec3(x, y, z)]) {
					//if (sqrt(particle->pos.squaredDistanceTo(p->pos)) <= h) { //! will be checked for later
					collisions.insert(particle);
					//}
				}
			}
		}
	}

	for (Particle* particle : collisions) {
		collisions_list.push_back(particle);
	}
	return collisions_list;
}

std::list<Particle*>& SpatialHash::operator[](const Vec3& pos) {

	return this->hashTable[this->hasher->Hash(pos, this->hashTable.size())];
}

size_t SpatialHash::next_prime(size_t minimum) {

outer:
	while (true) {
		minimum++;

		if (minimum == 2 || minimum == 3)
			return minimum;

		if (minimum % 2 == 0 || minimum % 3 == 0)
			continue;

		int divisor = 6;
		while (divisor * divisor - 2 * divisor + 1 <= minimum) {

			if (minimum % (divisor - 1) == 0
				|| minimum % (divisor + 1) == 0)
				goto outer;

			divisor += 6;

		}

		return minimum;

	}
}
