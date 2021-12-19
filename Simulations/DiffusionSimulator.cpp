#include "DiffusionSimulator.h"
#include "pcgsolver.h"
#include <random>
using namespace std;

Grid3d::Grid3d(int x, int y, int z)
{
	this->e2 = std::mt19937 (rd());
	this ->dist = std::uniform_real_distribution<Real> (-1.0, 1.0);
	this->setDimensions(std::make_tuple(x, y, z));
}

void Grid3d::setDimensions(std::tuple<int, int, int> dim3D) {
	this->m = std::get<0>(dim3D);
	this->n = std::get<1>(dim3D);
	this->p = std::get<2>(dim3D);

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

void Grid3d::resize(std::tuple<int, int, int> dim3D) {
	auto new_m = std::get<0>(dim3D);
	auto new_n = std::get<1>(dim3D);
	auto new_p = std::get<2>(dim3D);

	if (this->m == new_m && this->n == new_n && this->p == new_p) {
		return;
	}

	this->gridData.resize(new_m, std::vector<std::vector<Real>>(new_n, std::vector<Real>(new_p, 0)));

	for (auto& row : this->gridData) {
		row.resize(new_n, std::vector<Real>(new_p, 0));

		for (auto& column : row) {
			column.resize(new_p, 0);
		}
	}

	for (auto i = 0; i < new_m; ++i) {
		// n columns
		for (auto j = 0; j < new_n; ++j) {
			for (auto k =0; k < new_p; ++k) {
				if (i == 0 || j == 0 || k == 0 || (i == new_m - 1) || (j == new_n - 1) || (k == new_p - 1)) {
					this->gridData[i][j][k] = 0;
				}
				else {
					if (i >= this->m - 1 || j >= this->n-1 || k >= this->p-1) {
						this->gridData[i][j][k] = dist(e2);
					}
				}
			}
		}
	}

	this->m = new_m;
	this->n = new_n;
	this->p = new_p;
}

Grid::Grid(int x, int y) {
	this->e2 = std::mt19937(rd());
	this->dist = std::uniform_real_distribution<Real>(-1.0, 1.0);
	this->setDimensions(std::make_pair(x, y));
}

// Set grid dimensions to (x_dimension, y_dimension)
// Modifies m, n
void Grid::setDimensions(std::pair<int, int> dim2D) {
	this->m = dim2D.first;
	this->n = dim2D.second;

	// Initialize MxN matrix with 0 values
	this->gridData.resize(m, std::vector<Real>(n, 0));

	//set the value of each cell that is not a border
	for (auto i = 1; i < m - 1; ++i) {
		// n columns
		for (auto j = 1; j < n - 1; ++j) {
			this->gridData[i][j] = dist(e2);
		}
	}
}


void Grid::resize(std::pair<int, int> dim2D) {
	auto new_m = std::get<0>(dim2D);
	auto new_n = std::get<1>(dim2D);

	if (this->m == new_m && this->n == new_n) {
		return;
	}

	this->gridData.resize(new_m, std::vector<Real>(new_n, 0));

	for (auto& row : this->gridData) {
		row.resize(new_n, 0);
	}

	for (auto i = 0; i < new_m; ++i) {
		// n columns
		for (auto j = 0; j < new_n; ++j) {
				if (i == 0 || j == 0 || (i == new_m - 1) || (j == new_n - 1)) {
					this->gridData[i][j] = 0;
				}
				else {
					if (i >= this->m - 1 || j >= this->n - 1) {
						this->gridData[i][j] = dist(e2);
					}
				}
		}
	}

	this->m = new_m;
	this->n = new_n;
}


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
}

const char* DiffusionSimulator::getTestCasesStr() {
	return "Explicit_solver, Implicit_solver, 3D_Explicit_Solver, 3D_Implicit_Solver";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "alpha", TW_TYPE_FLOAT, &this->alpha, "min=0.3, max=1, step=0.1");
	TwAddVarRW(DUC->g_pTweakBar, "m", ETwType::TW_TYPE_UINT16, &this->_m, "min=3");
	TwAddVarRW(DUC->g_pTweakBar, "n", ETwType::TW_TYPE_UINT16, &this->_n, "min=3");
	TwAddVarRW(DUC->g_pTweakBar, "p", ETwType::TW_TYPE_UINT16, &this->_p, "min=3");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	switch (m_iTestCase)
	{
	case 0:
		delete d3T;
		delete T;

		d3T = nullptr;
		T = new Grid(_m, _n);

		cout << "Explicit solver!\n";
		break;
	case 1:
		delete d3T;
		delete T;

		d3T = nullptr;
		T = new Grid(_m, _n);
		cout << "Implicit solver!\n";
		break;
	case 2:
		delete d3T;
		delete T;

		d3T = new Grid3d(_m, _n, _p);
		T = nullptr;
		cout << "3D Explicit solver!\n";
		break;
	case 3:
		delete d3T;
		delete T;

		d3T = new Grid3d(_m, _n, _p);
		T = nullptr;
		cout << "3D Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}


// Returns a new grid calculated from the previous one
Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {//add your own parameters
	Grid* newT = new Grid(_m, _n);

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

			Parameters parameter{ data[index_row + 1][index_column],
				data[index_row][index_column - 1], data[index_row][index_column], data[index_row][index_column + 1],
				data[index_row - 1][index_column],1, 1, timeStep };

			newT->gridData[index_row][index_column] = this->updateValuesExplicit(parameter);
		}
	}

	return newT;
}

Real DiffusionSimulator::updateValuesExplicit(Parameters& param) {
	Real new_value = 0;

	Real x_second_deriv = (param.upperCenter - 2 * param.center + param.lowerCenter) / (param.deltaM * param.deltaM);
	Real y_second_deriv = (param.centerRight - 2 * param.center + param.centerLeft) / (param.deltaN * param.deltaN);

	new_value = param.center + this->alpha * (x_second_deriv + y_second_deriv) * param.timeStep;

	return new_value;
}


Grid3d* DiffusionSimulator::diffuseTemperatureExplicit3d(float timeStep) {

	Grid3d* newT = new Grid3d(_m, _n, _p);

	// Name alias
	std::vector<std::vector<std::vector<Real>>>& data = this->d3T->gridData;

	// Don't iterate on the border values
	auto row_len = data.size() - 1;
	auto column_len = data[0].size() - 1;
	auto depth_len = data[0][0].size() - 1;


	// For each value
	// Calculate new value in a new time using previous time
	// Positions used: upperCenter, lowerCenter, center(currentvalue), centerLeft, centerRight
	// DELTAx = DELTAy= DELTAz = 1
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


void DiffusionSimulator::setupB(std::vector<Real>& b) {
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < b.size(); i++) {
		b.at(i) = 0;
	}

	std::vector<std::vector<Real>>& data = this->T->gridData;

	for (int i = 0; i < data.size(); ++i) {
		for (int j = 0; j < data[0].size(); ++j) {
			b.at(i * data[0].size() + j) = data[i][j];
		}
	}

}

void DiffusionSimulator::fillT(std::vector<Real>& x) {
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	std::vector<std::vector<Real>>& data = this->T->gridData;

	const auto rows = data.size();
	const auto columns = data[0].size();

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < columns; ++j) {
			data[i][j] = x.at(i * columns + j);
			if (i == 0 || j == 0 || (i == rows - 1) || (j == columns - 1)) {
				data[i][j] = 0;
			}
		}
	}

}

void DiffusionSimulator::setupA(SparseMatrix<Real>& A, double factor) {
	// setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
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
				A.set_element(gridIndex, gridIndex, 1 + 4 * factor);
				A.set_element(gridIndex - 1, gridIndex, -factor);
				A.set_element(gridIndex + 1, gridIndex, -factor);
				A.set_element(gridIndex - columns, gridIndex, -factor);
				A.set_element(gridIndex + columns, gridIndex, -factor);
			}
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep) {//add your own parameters

	const int N = this->T->gridData.size() * this->T->gridData[0].size();//N = sizeX*sizeY

	SparseMatrix<Real>* A = new SparseMatrix<Real>(N);
	std::vector<Real>* b = new std::vector<Real>(N);

	// setupA(a, factor)
	// factor is this->alpha * timestep
	this->setupA(*A, this->alpha * timeStep);
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

				b[i * columns * depth + j * depth + k] = data[i][j][k];
			}
		}
	}

}

void DiffusionSimulator::fillT3d(std::vector<Real>& x) {//add your own parameters
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
				if (i == 0 || j == 0 || k == 0 || (i == rows - 1) || (j == columns - 1) || (k == depth - 1)) {
					A.set_element(gridIndex, gridIndex, 1);
				}
				else {
					A.set_element(gridIndex, gridIndex, 1 + 6 * factor);
					A.set_element(gridIndex - 1, gridIndex, -factor);
					A.set_element(gridIndex + 1, gridIndex, -factor);
					A.set_element(gridIndex - columns, gridIndex, -factor);
					A.set_element(gridIndex + columns, gridIndex, -factor);
					A.set_element(gridIndex - rows * columns, gridIndex, -factor);
					A.set_element(gridIndex + rows * columns, gridIndex, -factor);
				}
			}
		}
	}
}

void DiffusionSimulator::diffuseTemperatureImplicit3d(float timeStep) {

	const int N = this->d3T->gridData.size() * this->d3T->gridData[0].size() * this->d3T->gridData[0][0].size();//N = sizeX*sizeY*sizeZ

	SparseMatrix<Real>* A = new SparseMatrix<Real>(N);
	std::vector<Real>* b = new std::vector<Real>(N);

	// setupA(a, factor)
	// factor is this->alpha * timestep
	this->setupA3d(*A, this->alpha * timeStep);
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
	// update current setup for each frame

	switch (m_iTestCase) {
	case 0:
	{
		if (!this->T) {
			this->T = new Grid(this->_m, this->_n);
		}
		else {
			this->T->resize(std::make_pair(this->_m, this->_n));
		}
		Grid* temp = T;
		T = diffuseTemperatureExplicit(timeStep);
		delete temp;
		break;
	}
	case 1:
		if (!this->T) {
			this->T = new Grid(this->_m, this->_n);
		}
		else {
			this->T->resize(std::make_pair(this->_m, this->_n));
		}
		this->diffuseTemperatureImplicit(timeStep);
		break;
	case 2:
	{
		if (!this->d3T) {
			this->d3T = new Grid3d(_m, _n, _p);
		}
		else {
			this->d3T->resize(std::make_tuple(this->_m, this->_n, this->_p));
		}
		Grid3d* temp3D = d3T;
		d3T = diffuseTemperatureExplicit3d(timeStep);
		delete temp3D;
		break;
	}
	case 3:
		if (!this->d3T) {
			this->d3T = new Grid3d(_m, _n, _p);
		}
		else {
			this->d3T->resize(std::make_tuple(this->_m, this->_n, this->_p));
		}
		this->diffuseTemperatureImplicit3d(timeStep);
		break;
	default:
		fprintf(stderr, "No such cases!");
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// 2d visualization
	if (this->T) {
		auto& points = this->T->gridData;
		Real pointSize = 0.05f;

		for (size_t i = 0; i < points.size(); i++) {
			for (size_t j = 0; j < points[0].size(); j++) {

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

					DUC->drawSphere(Vec3(i / 14.0 - 0.5, j / 14.0 - 0.5, k / 14.0 - 0.5), Vec3(pointSize, pointSize, pointSize));
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