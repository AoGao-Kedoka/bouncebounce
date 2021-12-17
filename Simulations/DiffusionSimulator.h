#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"
#include "pcgsolver.h"
#include <vector>
#include <tuple>
class Grid3d {
public:
	// Construtors
	Grid3d(int x, int y, int z);

	std::vector<std::vector<std::vector<Real>>> gridData;
	void setDimensions(std::tuple<int, int, int> dim3D);
private:
	// Attributes
	int m;
	int n;
	int p;
};


//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid(int x, int y);

	std::vector<std::vector<Real>> gridData;
	void setDimensions(std::pair<int, int> dim2D);
private:
	// Attributes
	int m;
	int n;
};

class Parameters {
public:
	Parameters(Real upperCenter,Real centerLeft, Real center, Real centerRight, Real lowerCenter,
		int deltaM, int deltaN, Real timeStep):
		upperCenter{ upperCenter }, 
		centerLeft{ centerLeft }, center{ center }, centerRight{ centerRight },
		lowerCenter{ lowerCenter },
		deltaM{ deltaM }, deltaN { deltaN }, timeStep{timeStep}{}

	Real upperCenter;
	Real centerLeft, center, centerRight;
	Real lowerCenter;

	int deltaM, deltaN; 
	Real timeStep;
};

class Parameters3d{
public:
	Parameters3d(Real upperCenter, Real centerLeft, Real center, Real centerRight, 
				Real lowerCenter,Real forwardDepth, Real backwardDepth,
				int deltaM, int deltaN, int deltaP, Real timeStep):
		upperCenter{ upperCenter },
		centerLeft{ centerLeft }, center{ center }, centerRight{ centerRight },
		lowerCenter{ lowerCenter },
		forwardDepth{ forwardDepth }, backwardDepth{ backwardDepth },
		deltaM{ deltaM }, deltaN{ deltaN }, deltaP{ deltaP }, timeStep{ timeStep }{}

	Real upperCenter;
	Real centerLeft, center, centerRight;
	Real lowerCenter;
	Real forwardDepth, backwardDepth;

	int deltaM, deltaN, deltaP; 
	Real timeStep;
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

	Real updateValuesExplicit(Parameters& param);
	Real updateValuesExplicit3d(Parameters3d& param);

	Grid* diffuseTemperatureExplicit(float timeStep);
	Grid3d* diffuseTemperatureExplicit3d(float timeStep);

	void diffuseTemperatureImplicit(float timeStep);
	void diffuseTemperatureImplicit3d(float timeStep);


	//void assignValues(size_t index_row, size_t index_column, Parameters new_val);
	void setupA(SparseMatrix<Real>& A, double factor);
	void setupB(std::vector<Real>& b);
	void fillT(std::vector<Real>& x);

	void setupA3d(SparseMatrix<Real>& A, double factor);
	void setupB3d(std::vector<Real>& b);
	void fillT3d(std::vector<Real>& x);
private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	Grid *T = nullptr;
	Grid3d* d3T = nullptr;
	
	int _m = 16;
	int _n = 16;
	int _p = 16;

	float alpha = 0.3;//diffusion coef
};

#endif