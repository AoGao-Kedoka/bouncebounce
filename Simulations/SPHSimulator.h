#ifndef  SPHSIMULATOR_h
#define SPHSIMULATOR_h
#include "Simulator.h"
#include "Particle.h"

#define MIN_X -1
#define MAX_X 1
#define MIN_Y 0
#define EPS 0.001

class SPHSimulator :public Simulator {
public:
	// Constructors
	SPHSimulator(int width, int length, int height);
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	template<typename T=int>
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
	float _distance_between = 0.05;
};

#endif
