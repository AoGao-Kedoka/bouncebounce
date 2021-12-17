#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid(int m, int n) : _m{ m }, _n{ n } {
	std::random_device d;
	std::mt19937 rng(d());
	std::uniform_real_distribution<> dist(-1, 1);
	for (int i = 0; i < m; ++i) {
		std::vector<double> temp;
		for (int j = 0; j < n; ++j) {
			if (i == 0 || j == 0 || i == m - 1 || j == n - 1)
				temp.push_back(0);
			else {
				temp.push_back(dist(rng));
			}
		}
		_temperature.push_back(temp);
	}
}

int Grid::getM() { return _m; }
int Grid::getN() { return _n; }
float Grid::getTemperature(int l, int w) {
	if (l < _m || w < _n)
		return _temperature.at(l).at(w);
	else
		throw std::invalid_argument("m or n too big");
}
void Grid::setTemperatur(int l, int w, double value) {
	if (l < _m || w < _n)
		_temperature.at(l).at(w) = value;
	else
		throw std::invalid_argument("m or n too big");
}


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented
}

DiffusionSimulator::~DiffusionSimulator() {
	delete(T);
}

const char* DiffusionSimulator::getTestCasesStr() {
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	// to be implemented
	TwAddVarRW(DUC->g_pTweakBar, "alpha", TW_TYPE_DOUBLE, &this->alpha, "min=0, step=0.2");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		T = new Grid(10, 10);
		cout << "Explicit solver!\n";
		break;
	case 1:
		T = new Grid(10, 10);
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {//add your own parameters
	Grid* newT = new Grid(T->getM(), T->getN());
	for (int i = 0; i < T->getM(); ++i) {
		for (int j = 0; j < T->getN(); ++j) {
			if(i == 0 || j == 0 || i == T->getM() - 1 || j == T->getN() - 1){
				newT->setTemperatur(i, j, T->getTemperature(i, j));
			} else {
				/*
						  d^2u   d^2u
					t =	 (---- + ---- ) * timeStep * alpha + temperature
						  dx^2   dy^2
				*/
				float centralDiffX = T->getTemperature(i + 1, j) + T->getTemperature(i - 1, j) - 2 * T->getTemperature(i, j);
				float centralDiffY = T->getTemperature(i, j + 1) + T->getTemperature(i, j - 1) - 2 * T->getTemperature(i, j);
				newT->setTemperatur(i, j, T->getTemperature(i, j) + alpha * (centralDiffX + centralDiffY) * timeStep);
			}
		}
	}
	return newT;
}

void setupB(std::vector<Real>& b, Grid* T) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < T->getM() * T->getN(); i++) {
		b.at(i) = 0;
	}
}

void fillT(Grid* T) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
}

void setupA(SparseMatrix<Real>& A, double factor, Grid* T) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	int l = T->getM() * T->getN();
	for (int i = 0; i < l; ++i) {
		A.set_element(i, i, 1); // set diagonal
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep) {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = T->getN() * T->getM();//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real>* A = new SparseMatrix<Real>(N);
	std::vector<Real>* b = new std::vector<Real>(N);

	setupA(*A, 0.1, T);
	setupB(*b, T);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(T);//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0: {
		auto temp = T;
		T = diffuseTemperatureExplicit(timeStep);
		delete(temp);
		break;
	}
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	int m = T->getM();
	int n = T->getN();
	for (int i = 0; i < m; ++i) {
		for (int j = 0; j < n; ++j) {
			if (i == 0 || j == 0 || i == m - 1 || j == n - 1) {
				DUC->setUpLighting(Vec3(), Vec3(0, 0, 0), 10, Vec3(0, 0, 0));
			}
			else {
				double t = T->getTemperature(i, j);
				DUC->setUpLighting(abs(T->getTemperature(i,j)) * 
									Vec3(1, T->getTemperature(i, j)>0? 1:0, T->getTemperature(i, j) > 0 ? 1 : 0), 
									0.4 * Vec3(1, 1, 1),10,
									abs(T->getTemperature(i, j)) * Vec3(1,1,1));
			}
			DUC->drawSphere(Vec3(0.05 * (i - m / 2), 0.05 * (j - n / 2), 0), Vec3(0.05, 0.05, 0.05));
		}
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
