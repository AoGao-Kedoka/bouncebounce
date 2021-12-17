#include "DiffusionSimulator.h"
#include "pcgsolver.h"
#include <random>
using namespace std;

Grid3d::Grid3d(int x, int y, int z) {
	this->setDimensions(std::make_tuple(x, y, z));
}

void Grid3d::setDimensions(std::tuple<int, int, int> dim3D) {
	this->m = std::get<0>(dim3D);
	this->n = std::get<1>(dim3D);
	this->p = std::get<2>(dim3D);

	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<Real> dist(-1.0, 1.0);


	this->gridData.resize(m, std::vector<std::vector<Real>>(n, std::vector<Real>(p, 0)));

	for (auto i = 1; i < m - 1; ++i) {
		// n columns
		for (auto j = 1; j < n - 1; ++j) {
			for (auto k = 1; k < p - 1; ++k) {
				this->gridData[i][j][k] = dist(e2);
			}
		}
	}

}


Grid::Grid(int x, int y) {
	this->setDimensions(std::make_pair(x, y));
}

// Set grid dimensions to (x_dimension, y_dimension)
// Modifies m, n
void Grid::setDimensions(std::pair<int, int> dim2D) {
	this->m = dim2D.first;
	this->n = dim2D.second;

	// Random values in range [-1.0, 1.0]
	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<Real> dist(-1.0, 1.0);

	// Initialize MxN matrix with 0 values
	this->gridData.resize(m, std::vector<Real>(n, 0));
	
	//set the value of each cell that is not a border
	for (auto i = 1; i < m-1; ++i) {
		// n columns
		for (auto j = 1; j < n-1; ++j) {
			this->gridData[i][j] = dist(e2) ;
		}
	}
}


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented
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


// Returns a new grid calculated from the previous one
Grid* DiffusionSimulator::diffuseTemperatureExplicit() {//add your own parameters
	Real timeStep = 0.1;

	Grid* newT = new Grid();

	// Name alias
	std::vector<std::vector<Real>>& data = this->T->gridData;

	// Don't iterate on the border values
	auto row_len = data.size() - 1;
	auto column_len = data[0].size() - 1;

	// For each value
	// Calculate new value in a new time using previous time
	// Positions used: upperCenter, lowerCenter, center(currentvalue), centerLeft, centerRight
	// DELTAx = DELTAy = 1
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

	return newT;//leak
}

Real DiffusionSimulator::updateValuesExplicit(Parameters& param) {
	Real new_value = 0;

	Real x_second_deriv = (param.upperCenter - 2 * param.center + param.lowerCenter) / (param.deltaM * param.deltaM);
	Real y_second_deriv = (param.centerRight - 2 * param.center + param.centerLeft) / (param.deltaN * param.deltaN);

	new_value = param.center + this->alpha * (x_second_deriv + y_second_deriv) * param.timeStep;

	return new_value;
}


Grid3d* DiffusionSimulator::diffuseTemperatureExplicit3d() {
	Real timeStep = 0.1;

	Grid3d* newT = new Grid3d();

	// Name alias
	std::vector<std::vector<std::vector<Real>>>& data = this->d3T->gridData;

	// Don't iterate on the border values
	auto row_len = data.size() - 1;
	auto column_len = data[0].size() - 1;
	auto depth_len = data[0][0].size() - 1;


	// For each value
	// Calculate new value in a new time using previous time
	// Positions used: upperCenter, lowerCenter, center(currentvalue), centerLeft, centerRight
	// DELTAx = DELTAy= DELTAy = 1
	for (auto index_row = 1; index_row < row_len; index_row++) {
		for (auto index_column = 1; index_column < column_len; index_column++) {
			for (auto index_depth = 1; index_depth < depth_len; index_depth++) {
				Parameters3d parameter{
					data[index_row - 1][index_column][index_depth],
					data[index_row][index_column - 1][index_depth], 
					data[index_row][index_column][index_depth], 
					data[index_row][index_column + 1][index_depth],
					data[index_row + 1][index_column][index_depth],
					data[index_row][index_column][index_depth + 1],
					data[index_row][index_column][index_depth - 1],
					1, 1, 1, timeStep
			};

				newT->gridData[index_row][index_column][index_depth] = this->updateValuesExplicit3d(parameter);
			}
		}
	}

	return newT;//leak
}

Real DiffusionSimulator::updateValuesExplicit3d(Parameters3d& param) {
	Real new_value = 0;

	Real x_second_deriv = (param.upperCenter - 2 * param.center + param.lowerCenter) / (param.deltaM * param.deltaM);
	Real y_second_deriv = (param.centerRight - 2 * param.center + param.centerLeft) / (param.deltaN * param.deltaN);
	Real z_second_deriv = (param.forwardDepth - 2 * param.center + param.backwardDepth) / (param.deltaP * param.deltaP);

	new_value = param.center + this->alpha * (x_second_deriv + y_second_deriv + z_second_deriv) * param.timeStep;

	return new_value;
}


void DiffusionSimulator::setupB(std::vector<Real>& b) {//add your own parameters
	
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < b.size(); i++) {
		b.at(i) = 0;
	}

	std::vector<std::vector<Real>>& data = this->T->gridData;

	for (int i = 0; i < data.size() ; ++i) {
		for (int j = 0; j < data[0].size() ; ++j) {
			b.at(i* data[0].size() +j) = data[i][j];
		}
	}
	
}

void DiffusionSimulator::fillT(std::vector<Real>& x) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	std::vector<std::vector<Real>>& data = this->T->gridData;

	const auto rows = data.size();
	const auto columns = data[0].size();

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; columns; ++j) {
			data[i][j] = x.at(i* columns +j);

			if (i == 0 || j == 0 || (i == rows - 1) || (j == columns - 1)) {
				data[i][j] = 0;
			}
		}
	}

}

void DiffusionSimulator::setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0

	std::vector<std::vector<Real>>& data = this->T->gridData;
	const int N = data.size() * data[0].size();//N = sizeX*sizeY*sizeZ

	const auto rows = data.size();
	const auto columns = data[0].size();

	A.zero();

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < columns; ++j) {
			const auto gridIndex = i * columns + j;

			if (i == 0 || j == 0 || (i == rows - 1) || (j == columns - 1)) {
				A.set_element(gridIndex, gridIndex, 1);
			}
			else {
				A.set_element(gridIndex, gridIndex, 1+4*factor);
				A.set_element(gridIndex - 1, gridIndex, -factor);
				A.set_element(gridIndex + 1, gridIndex, -factor);
				A.set_element(gridIndex - columns, gridIndex, -factor);
				A.set_element(gridIndex + columns, gridIndex, -factor);
			}
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit() {//add your own parameters

	const int N = this->T->gridData.size() * this->T->gridData[0].size();//N = sizeX*sizeY

	SparseMatrix<Real> *A = new SparseMatrix<Real> (N); 
	std::vector<Real> *b = new std::vector<Real>(N);	

	// setupA(a, factor)
	// factor is this->alpha * timestep
	this->setupA(*A, 0.005);
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
	fillT(x);//copy x to T
	delete A;
	delete b;
}



void DiffusionSimulator::setupB3d(std::vector<Real>& b) {//add your own parameters

	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < b.size(); i++) {
		b.at(i) = 0;
	}
	std::vector< std::vector<std::vector<Real>>>& data = this->d3T->gridData;

	const auto rows = data.size();
	const auto columns = data[0].size();
	const auto depth = data[0][0].size();
	
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < columns; ++j) {
			for (int k = 0; k < depth; ++k) {

				b[i *columns*depth + j*depth+ k] = data[i][j][k];
			}
		}
	}
	
}

void DiffusionSimulator::fillT3d(std::vector<Real>& x) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	std::vector< std::vector<std::vector<Real>>>& data = this->d3T->gridData;

	const auto rows = data.size();
	const auto columns = data[0].size();
	const auto depth = data[0][0].size();

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < columns; ++j) {
			for (int k = 0; k < depth; ++k) {

				if (i == 0 || j == 0 || k == 0 || (i == rows - 1) || (j == columns - 1) || (k == depth - 1)) {
					data[i][j][k] = 0;
				}
				else {
					data[i][j][k] = x.at(i * columns * depth + j * depth + k);
				}
			}
		}
	}

}

void DiffusionSimulator::setupA3d(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0

	std::vector<std::vector<std::vector<Real>>>& data = this->d3T->gridData;

	const auto rows = data.size();
	const auto columns = data[0].size();
	const auto depth = data[0][0].size();
	A.zero();

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < columns; ++j) {
			for (int k = 0; k < depth; ++k) {
				const auto gridIndex = i * columns * depth + j * depth + k;
				if (i == 0 || j == 0 || k == 0 || (i == rows - 1) || (j == columns- 1) || (k == depth - 1)) {
					A.set_element(gridIndex, gridIndex, 1);
				}
				else {
					A.set_element(gridIndex, gridIndex, 1 + 6 * factor);
					A.set_element(gridIndex - 1, gridIndex, -factor);
					A.set_element(gridIndex + 1, gridIndex, -factor);
					A.set_element(gridIndex - columns, gridIndex, -factor);
					A.set_element(gridIndex + columns, gridIndex, -factor);
					A.set_element(gridIndex - depth, gridIndex, -factor);
					A.set_element(gridIndex + depth, gridIndex, -factor);
				}
			}
		}
	}
}

void DiffusionSimulator::diffuseTemperatureImplicit3d() {

	const int N = this->d3T->gridData.size() * this->d3T->gridData[0].size() * this->d3T->gridData[0][0].size();//N = sizeX*sizeY*sizeZ

	SparseMatrix<Real>* A = new SparseMatrix<Real>(N);
	std::vector<Real>* b = new std::vector<Real>(N);
	
	// setupA(a, factor)
	// factor is this->alpha * timestep
	this->setupA3d(*A, 0.01);
	setupB3d(*b);
	
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
	fillT3d(x);//copy x to T
	delete A;
	delete b;
}




void DiffusionSimulator::simulateTimestep(float timeStep) {
	// to be implemented
	// update current setup for each frame
	/*
	if (!this->d3T) this->d3T = new Grid3d();
	Grid3d* temp = diffuseTemperatureExplicit3d();
	delete this->d3T;
	this->d3T = temp;
	*/
	//if (!this->d3T) this->d3T = new Grid3d();
	//this-> diffuseTemperatureImplicit3d();	
	/*if (!this->T) this->T = new Grid();
	this->diffuseTemperatureImplicit();*/

	/*switch (m_iTestCase) {
	case 0: {
		 
		}
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}*/
}

void DiffusionSimulator::drawObjects()
{
	// 2d visualization
	if (this->T){
		auto& points = this->T->gridData;
		Real pointSize = 0.05f;

		for (size_t i = 0; i < points.size(); i++) {
			for (size_t j = 0; j < points[0].size() ; j++) {

				DUC->setUpLighting(abs(points[i][j]) * Vec3(1, points[i][j] > 0 ? 1 : 0, points[i][j] > 0 ? 1 : 0),
					0.4 * Vec3(1, 1, 1),
					10, abs(points[i][j]) * Vec3(1, 1, 1));

				DUC->drawSphere(Vec3(i / 14.0 - 0.5, j / 14.0 - 0.5, 0), Vec3(pointSize, pointSize, pointSize));

			}
		}
	}
	else if (this->d3T) {// 3d
		
		auto& points = this->d3T->gridData;
		Real pointSize = 0.05f;

		for (size_t i = 1; i < points.size() - 1; i++) {
			for (size_t j = 1; j < points[0].size() - 1; j++) {
				for (size_t k = 1; k < points[0][0].size() - 1; k++) {

					DUC->setUpLighting(abs(points[i][j][k]) * Vec3(1, points[i][j][k] > 0 ? 1 : 0, points[i][j][k] > 0 ? 1 : 0),
						0.4 * Vec3(1, 1, 1),
						10, abs(points[i][j][k]) * Vec3(1, 1, 1));

					DUC->drawSphere(Vec3(i / 14.0 - 0.5, j / 14.0 - 0.5,  k /14.0 -0.5), Vec3(pointSize, pointSize, pointSize));
				}
			}
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
