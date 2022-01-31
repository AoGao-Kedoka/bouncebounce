#ifndef  SPHSIMULATOR_h
#define SPHSIMULATOR_h
#include "Simulator.h"
#include "Particle.h"

#define MIN_X -0.2
#define MAX_X 0.2
#define MIN_Y 0
#define MAX_Y 0.4
#define EPS 0.001

#define STATE_WATER 0
#define STATE_MUCUS 1
#define STATE_GAS 2

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

	int CURR_STATE = STATE_WATER;

	void setConstants(int state);
	// kernel radius
	float H = 0.0457f;
	// mass of particles
	float M = 0.02f;
	// viscosity constant
	float MU = 3.5f;
	// rest density
	float REST_DENS = 998.29f;
	// Buoyancy; 0 for not gas
	float B = 0;

	// const for equation of state
	static constexpr float GAS_CONST = 3.0f;

	// (kernel radius)^2 for optimization
	float H2 = H * H;
	float PI = 3.1415926535f;
	// smoothing kernel as described in the Müller paper
	float H9 = H * H * H * H * H * H * H * H * H;
	float H6 = H * H * H * H * H * H;
	float POLY6 = 315.f / (64.f * PI * H9);
	float POLY6_GRAD_PRESS = -45.f / (PI * H6);
	float POLY6_GRAD_VISC = 45.f / (PI * H6);


	// -----------------------------
	// MASS SPRING SYSTEM
	// -----------------------------

	// toogles the Mass Spring System simulation
	bool coupleSimulation = false;

	struct MassPoint {
		Vec3 p;
		Vec3 v;
		bool isFixed;
		MassPoint(Vec3 p, Vec3 v, bool isFixed) : p(p), v(v), isFixed(isFixed) {};
	};
	// Spring connecting Mass Points A and B
	struct Spring {
		int A;
		int B;
		// initial length
		float L;
		Spring(int A, int B, float L) : A(A), B(B), L(L) {};
	};
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	float m_fRestLength;
	bool m_bIsGravity;
	Vec3 m_vGravityValue;
	float m_fTimeStep;
	float m_fRadius = 0.05;

	// Containers
	std::vector<MassPoint> masspoints;
	std::vector<Spring> springs;

	void initSprings();
	float calcDist(Vec3 A, Vec3 B, Vec3* dir);
	std::pair<Vec3, Vec3> calcAcc(Vec3 A, Vec3 B);
	void computeEuler(float timeStep);
	void computeMidPoint(float timeStep);
	void computeLeapFrog(float timeStep);
	void writeState();

	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	void setGravityValue(Vec3 g);
	void setGravityValue(float x, float y, float z);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
};

#endif
