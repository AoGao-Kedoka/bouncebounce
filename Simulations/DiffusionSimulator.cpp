#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid() {
	this->setDimensions(std::make_pair(16, 16));
}

void Grid::setDimensions(std::pair<int, int> dim2D) {
	this->m = dim2D.first;
	this->n = dim2D.second;

	this->gridData.resize(m);
	for (auto index = 0; index < this->gridData.size(); index++) {
		this->gridData[index] = std::vector<float>{};
		this->gridData[index].resize(n);
	}
}


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented

	this->T = new Grid();
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
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
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit() {//add your own parameters
	float timeStep = 0.1;
	Grid* newT = new Grid();
	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	// Name alias
	std::vector<std::vector<float>>& data = this->T->gridData;
	if (!called) { 
		called = true; 
		//data[9][9] = 100; 
		data[1][1] = -200;
	}
	// Don't iterate on the border values
	auto row_len = data.size() - 1;
	auto column_len = data[0].size() - 1;

	for (auto index_row = 1; index_row < row_len; index_row++) {
		for (auto index_column = 1; index_column < column_len; index_column++) {

			Parameters parameter{data[index_row + 1][index_column],
														data[index_row][index_column - 1], data[index_row][index_column], data[index_row][index_column +1],
														data[index_row - 1][index_column],
														1, 1, timeStep
													 };

			newT->gridData[index_row][index_column] = this->updateValuesExplicit(parameter);

		}
	}

	/*std::cout << "New value:" << std::endl;
	for (auto index_row = 0; index_row <= row_len; index_row++) {
		for (auto index_column = 0; index_column <= column_len; index_column++) {
			std::cout << data[index_row][index_column] << "  ";
		}
		std::cout << std::endl;
	}*/

	return newT;//leak
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}
}

void fillT() {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
}

void setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
			A.set_element(i, i, 1); // set diagonal
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit() {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = 25;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, 0.1);
	setupB(*b);

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
	fillT();//copy x to T
}

/*void DiffusionSimulator::assignValues(size_t index_row, size_t index_column, Parameters& param) {
		std::vector<std::vector<float>>& data = this->T->gridData;
		data[index_row + 1][index_column] = param.upperCenter;
		data[index_row][index_column - 1] = param.centerLeft;
		data[index_row][index_column] = param.center;
		data[index_row][index_column + 1] = param.centerRight;
		data[index_row - 1][index_column] = param.lowerLeft;
}*/



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit();
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	auto& points = this->T->gridData;
	float pointSize = 0.05f;

	for (size_t i = 0; i < points.size(); i++) {
		for (size_t j = 0; j < points.size(); j++) {

			DUC->setUpLighting(abs(points[i][j]) * Vec3(1, points[i][j] > 0 ? 1 : 0, points[i][j] > 0 ? 1 : 0),
				0.4 * Vec3(1, 1, 1),
				10, abs(points[i][j]) * Vec3(1, 1, 1));

			DUC->drawSphere(Vec3(i/15.0-0.5, j/15.0-0.5, 0), Vec3(pointSize, pointSize, pointSize));

		}
	}

}


float DiffusionSimulator::updateValuesExplicit(Parameters& param) {
	float new_value = 0;

	float x_second_deriv = (param.upperCenter - 2 * param.center + param.lowerCenter) / (param.deltaM * param.deltaM);
	float y_second_deriv = (param.centerRight - 2*param.center + param.centerLeft)/(param.deltaN * param.deltaN);

	new_value = param.center + alpha * (x_second_deriv + y_second_deriv) * param.timeStep;

	return new_value;
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
