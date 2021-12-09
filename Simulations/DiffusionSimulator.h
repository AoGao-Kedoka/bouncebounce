#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"
#include <vector>

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid();

	std::vector<std::vector<float>> gridData;
	void setDimensions(std::pair<int, int> dim2D);
private:
	// Attributes
	int m;
	int n;
};

class Parameters {
public:
	Parameters(							float upperCenter,
						float centerLeft, float center, float centerRight,
													float lowerCenter,
						int deltaM,      int deltaN, float timeStep
					) :
		upperCenter{ upperCenter }, 
		centerLeft{ centerLeft }, center{ center }, centerRight{ centerRight },
		lowerCenter{ lowerCenter },
		deltaM{ deltaM }, deltaN { deltaN }, timeStep{timeStep}
{}

	float upperCenter;
	float centerLeft;	float center;	float centerRight;
	float lowerCenter;

	int deltaM; int deltaN; float timeStep;
};


class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	float updateValuesExplicit(Parameters& param);
	Grid* diffuseTemperatureExplicit();
	void diffuseTemperatureImplicit();

	//void assignValues(size_t index_row, size_t index_column, Parameters new_val);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid *T; //save results of every time step
	float alpha = 0.3;//diffusion coef
	bool called = false;
};

#endif