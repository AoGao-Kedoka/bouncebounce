#ifndef  SPHSIMULATOR_h
#define SPHSIMULATOR_h
#include "Simulator.h"
#include "Particle.h"

#define MIN_X -0.2
#define MAX_X 0.2
#define MIN_Y 0
#define EPS 0.001

class SPHSimulator :public Simulator {
public:
	// Constructors
	SPHSimulator(int width, int length, int height);
	~SPHSimulator();
	// Functions
	void buildParticles(int width, int length, int height);
	void cleanupParticles();
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
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

	constexpr int flatten(int x, int y, int z) const
	{
		return z + _height * y + _height * _length * x;
	}


private:
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

#endif
