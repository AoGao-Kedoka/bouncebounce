#ifndef  SPHSIMULATOR_h
#define SPHSIMULATOR_h
#include "Simulator.h"
#include "Particle.h"

#include <list>
#include <vector>

#define MIN_X -0.2
#define MAX_X 0.2
#define MIN_Y 0
#define EPS 0.001

class SpatialHash;

class SPHSimulator :public Simulator {
public:
	// Constructors
	SPHSimulator(int width, int length, int height);
	~SPHSimulator();
	// Functions
	void buildParticles(int width, int length, int height);
	void cleanupParticles();
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);

	void computeDensityAndPressure(Particle* p);
	void computeForces(Particle* p);
	void computeLocation(Particle* p, float timeStep);

	void onClick(int x, int y);
	void onMouse(int x, int y);

	constexpr int flatten(int x, int y, int z) const {
		return z + _height * y + _height * _length * x;
	}


private:
	SpatialHash* spatialHash;

	Vec3 externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	int _width;
	int _length;
	int _height;
	std::vector<Particle> particles;
	float _distance_between = 0.01;

	// rest density
	static constexpr float REST_DENS = 1000.0f;
	// const for equation of state
	static constexpr float GAS_CONST = 3.0f;
	// kernel radius
	//static constexpr float H = 0.0457f;
	//static constexpr float H = 0.03f;
	static constexpr float H = 0.0726f;
	// (kernel radius)^2 for optimization
	static constexpr float H2 = H * H;
	// mass of particles
	static constexpr float M = 0.04f;
	// viscosity constant
	//static constexpr float MU = 42.f;
	static constexpr float MU = 36.f;
	static constexpr float PI = 3.1415926535f;
	// smoothing kernel as described in the Müller paper
	static constexpr float H9 = H * H * H * H * H * H * H * H * H;
	static constexpr float H6 = H * H * H * H * H * H;
	static constexpr float POLY6 = 315.f / (64.f * PI * H9);
	static constexpr float POLY6_GRAD_PRESS = -45.f / (PI * H6);
	static constexpr float POLY6_GRAD_VISC = 45.f / (PI * H6);
};

/////////////////////////////////////////////////////// HELPER FUNCTIONS
struct Discretize {
	Discretize(float radius) : radius(radius) {}  // Constructor
	int operator()(float dimension) const {
		return floor(static_cast<double>(dimension) / radius);
	}

	nVec3i operator()(Vec3 dimensions) const {
		return nVec3i{
			(*this)(dimensions.x),
			(*this)(dimensions.y),
			(*this)(dimensions.z)
		};
	}
private:
	float radius;
};


/////////////////////////////////////////////////////// HASHER

class IHasher {
public:
	virtual size_t Hash(const Vec3& pos, size_t length) = 0;
};

class ParticleHasher : public IHasher {
public:
	ParticleHasher(float radius) : discretize{ radius } {};
	virtual size_t Hash(const Vec3& pos, size_t length) override {
		int rx = discretize(pos.x);
		int ry = discretize(pos.y);	
		int rz = discretize(pos.z);

		static constexpr auto p1 = 73856093;
		static constexpr auto p2 = 19349663;
		static constexpr auto p3 = 83492791;

		return 
			((rx * p1) ^ (ry * p2) ^ (rz * p3))
			 % length;
	}
private:
	Discretize discretize;
};

/////////////////////////////////////////////////////// SPATIAL HASH

class SpatialHash {
public:
		SpatialHash(const std::vector<Particle>& particles, float h);
		std::vector<const Particle*> collisions(const Particle* p);

		std::vector<const Particle*>& operator[](const Vec3&);
		void reset(const std::vector<Particle>& particles);

		std::vector<std::vector<const Particle*>> hashTable;
	private:
		float h; //smoothing kernel radius
		Discretize discretize;
		IHasher* hasher;

		size_t next_prime(size_t minimum);

		size_t previous_size;
		size_t previous_prime;
};


#endif
