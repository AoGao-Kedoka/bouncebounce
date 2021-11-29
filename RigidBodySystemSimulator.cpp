#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return nullptr;
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	
	this->DUC = DUC;
	TwRemoveVar(DUC->g_pTweakBar, "Timestep");
	TwAddVarRW(DUC->g_pTweakBar, "Timestep", TW_TYPE_FLOAT, &this->timestep, "min=0,step = 0.001");
	TwAddSeparator(DUC->g_pTweakBar, "SeperatorUp", "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &this->gravityToggle, "");
	if (m_iTestCase == 2 || m_iTestCase == 3 || m_iTestCase == 5) {
		TwAddButton(DUC->g_pTweakBar, "Info", NULL, NULL, "label='0=Euler,1=Leap-Frog,2=Midpoint'");
		TwAddButton(DUC->g_pTweakBar, "Info1", NULL, NULL, "label='0=Euler'");
		TwAddButton(DUC->g_pTweakBar, "Info2", NULL, NULL, "label='1=Leap-Frog'");
		TwAddButton(DUC->g_pTweakBar, "Info3", NULL, NULL, "label='2=Midpoint'");
	}
	/*else if (m_iTestCase == 4) {
		TwAddButton(DUC->g_pTweakBar, "Info1", NULL, NULL, "label='0=Euler'");
		TwAddButton(DUC->g_pTweakBar, "Info2", NULL, NULL, "label='1=Leap-Frog'");
		TwAddButton(DUC->g_pTweakBar, "Info3", NULL, NULL, "label='2=Midpoint'");
		TwRemoveVar(DUC->g_pTweakBar, "Integrate");
		TwAddButton(DUC->g_pTweakBar, "InfoComplex", NULL, NULL, "label='0=Euler,1=Leap-Frog,2=Midpoint'");
		//TwAddVarRW(DUC->g_pTweakBar, "Integrate", TW_TYPE_INT8, &m_iIntegrator, "min=0, max=2");
		//TwRemoveVar(DUC->g_pTweakBar, "Timestep");
		//TwAddVarRW(DUC->g_pTweakBar, "Timestep", TW_TYPE_FLOAT, &m_ftimeStep_Cur, "min=0,step = 0.001");
	}
	*/
}

void RigidBodySystemSimulator::reset()
{
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (const auto& body : this->rigidBodies) {
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		Mat4 rot = this->orientations[0].getRotMat();
		Mat4 trans; trans.initTranslation(std::get<0>(body).x, std::get<0>(body).y, std::get<0>(body).z);
		Mat4 scale; scale.initScaling(std::get<1>(body).x, std::get<1>(body).y, std::get<1>(body).z);
		Mat4 BodyA = rot * trans * scale;//BodyA.translatMat
		DUC->drawRigidBody(BodyA);
		std::cout << std::get<0>(this->rigidBodies[0]);
	}

}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase) {
	case 1:
		std::cout << "Demo0: Demo for test usage" << std::endl;

		break;
	case 0:
		this->m_iTestCase = TESTCASEUSEDTORUNTEST;
		this->addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		this->setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));
		this->applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));

		this->addRigidBody(Vec3(1.0f, 1.0f, 1.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		this->setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));

		std::cout << "Demo 1: Simple   single   body   simulation " << std::endl;
		break;
	}

	/*case 2:
		this->buildSprings(1);
		m_iIntegrator = EULER;
		std::cout << "Demo 2: A simple Euler simulation" << std::endl;
		break;
	case 3:
		m_iIntegrator = MIDPOINT;
		this->buildSprings(1);
		std::cout << "Demo 3: A simple Midpoint simulation" << std::endl;
		break;
	case 4:
		m_iIntegrator = EULER;
		this->buildSprings(10);
		std::cout << "Demo 4: A complex simulation, comparing stability of Euler and Midpoint" << std::endl;
		break;
	case 5:
		m_iIntegrator = LEAPFROG;
		this->buildSprings(1);
		std::cout << "Demo 5: A simple Leap-Frog simulation" << std::endl;
		break;
	default:
		break;
	}*/
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0) {
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.00001f;
		inputWorld = inputWorld * inputScale;
		for (int i = 0; i < this->rigidBodies.size(); i++)
			applyForceOnBody(0, inputWorld,Vec3(100,100,100));
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	this->simulateWorld(timeStep);

}

void RigidBodySystemSimulator::onClick(int x, int y)
{

	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{

	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return this->rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return std::get<0>(this->rigidBodies[i]);
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return this->velocities[i];
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return this->angularVelocities[i];
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	this->torques[i] = crossproduct(loc, force);
	this->forces[i] += force;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	this->rigidBodies.emplace_back(position, size, mass);
	this->orientations.emplace_back();
	this->velocities.emplace_back();
	this->angularVelocities.emplace_back();
	this->angularMomentum.emplace_back();
	this->torques.emplace_back();	
	this->tensors.emplace_back();
	this->masses.emplace_back();
	this->forces.emplace_back();
	this->bounciness.emplace_back();
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	this->orientations[i] = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	this->velocities[i] = velocity;
}

Mat4 RigidBodySystemSimulator::calculateTensor(const std::tuple<Vec3, Vec3, int>& rigidBody)
{
	/*
	std::vector<Vec3> points {
		Vec3{std::get<0>(rigidBody).x + std::get<1>(rigidBody).x/2, std::get<0>(rigidBody).y + std::get<1>(rigidBody).y/2, std::get<0>(rigidBody).z - std::get<1>(rigidBody).z/2},
		Vec3{std::get<0>(rigidBody).x + std::get<1>(rigidBody).x/2, std::get<0>(rigidBody).y - std::get<1>(rigidBody).y/2, std::get<0>(rigidBody).z - std::get<1>(rigidBody).z/2},
		Vec3{std::get<0>(rigidBody).x + std::get<1>(rigidBody).x/2, std::get<0>(rigidBody).y - std::get<1>(rigidBody).y/2, std::get<0>(rigidBody).z + std::get<1>(rigidBody).z/2},
		Vec3{std::get<0>(rigidBody).x + std::get<1>(rigidBody).x/2, std::get<0>(rigidBody).y + std::get<1>(rigidBody).y/2, std::get<0>(rigidBody).z + std::get<1>(rigidBody).z/2},
		Vec3{std::get<0>(rigidBody).x - std::get<1>(rigidBody).x/2, std::get<0>(rigidBody).y + std::get<1>(rigidBody).y/2, std::get<0>(rigidBody).z + std::get<1>(rigidBody).z/2},
		Vec3{std::get<0>(rigidBody).x - std::get<1>(rigidBody).x/2, std::get<0>(rigidBody).y + std::get<1>(rigidBody).y/2, std::get<0>(rigidBody).z + std::get<1>(rigidBody).z/2},
		Vec3{std::get<0>(rigidBody).x - std::get<1>(rigidBody).x/2, std::get<0>(rigidBody).y - std::get<1>(rigidBody).y/2, std::get<0>(rigidBody).z - std::get<1>(rigidBody).z/2},
		Vec3{std::get<0>(rigidBody).x - std::get<1>(rigidBody).x/2, std::get<0>(rigidBody).y - std::get<1>(rigidBody).y/2, std::get<0>(rigidBody).z - std::get<1>(rigidBody).z/2}
	};
	
	float matrix_x = 0;
	float matriy_y = 0;
	float matriz_z = 0;
	
	for (auto& point : points) {
		matrix_x += point.x * point.x * std::get<2>(rigidBody)/8;//?
		matriy_y += point.y * point.y * std::get<2>(rigidBody)/8;
		matriz_z += point.z * point.z * std::get<2>(rigidBody)/8;
	
	}
	Mat4 matrix = { 
					matrix_x,	0.0f,	  0.0f,		0.0f,
					0.0f,		matriy_y, 0.0f,		0.0f,
					0.0f,		0.0f,	  matriz_z, 0.0f,
					0.0f,		0.0f,	  0.0f,		1.0f
					};
	*/

	auto weight_div = 1 / 12 * std::get<2>(rigidBody);
	auto size = std::get<1>(rigidBody);
	Mat4 matrix = {
					weight_div*(size.z*size.z* (size.y*size.y)),	0.0f,	  0.0f,		0.0f,
					0.0f,		weight_div* (size.x * size.x * (size.z * size.z)), 0.0f,		0.0f,
					0.0f,		0.0f,	  weight_div* (size.x * size.x * (size.y * size.y)), 0.0f,
					0.0f,		0.0f,	  0.0f,		1.0f
	};

	/*float trace = matrix. + matrix[1][1] + matrix[2][2];
	return Mat4{ 1.0, 0.0, 0.0, 0.0,
							0.0, 1.0, 0.0, 0.0,
							0.0, 0.0, 1.0, 0.0,
							0.0, 0.0, 0.0, 1.0
						  } * trace - (matrix);
	*/
	return matrix;
}

Vec3 RigidBodySystemSimulator::crossproduct(const Vec3& V1, const Vec3& V2)
{
	return Vec3{
		(V1.y * V2.z) - (V1.z * V2.y),
		(V1.z * V2.x) - (V1.x * V2.z),
		(V1.x * V2.y) - (V1.y * V2.x)
	};
}

void RigidBodySystemSimulator::init() {
	for (int i = 0; i < this->rigidBodies.size(); i++) {
		this->tensors[i] = this->calculateTensor(this->rigidBodies[i]);
		auto temp = this->orientations[i].getRotMat();
		temp.transpose();
		Mat4 inertiaTensorInv = this->orientations[i].getRotMat() * (this->tensors[i].inverse() * temp);
		this->angularMomentum[i] = inertiaTensorInv * this->angularMomentum[i];

	}
	this->isInit = true;
}


Vec3 RigidBodySystemSimulator::calculateImpulse(int bodyA, int bodyB, CollisionInfo& info) {
	auto temp_point_a = info.collisionPointWorld - std::get<0>(this->rigidBodies[bodyA]);
	auto temp_point_b = info.collisionPointWorld - std::get<0>(this->rigidBodies[bodyB]);

	auto temp_vel_a = this->velocities[bodyA] + this->angularVelocities[bodyA] * temp_point_a;
	auto temp_vel_b = this->velocities[bodyA] + this->angularVelocities[bodyB] * temp_point_b;

	auto rel_vel = temp_vel_a - temp_vel_b;

	Vec3 denominator = -(1.0 + this->bounciness[bodyA]) * rel_vel * info.normalWorld;
	Vec3 nominator = (1.0/std::get<2>(this->rigidBodies[bodyA])) + (1.0 / std::get<2>(this->rigidBodies[bodyB])) +
		(crossproduct(this->tensors[bodyA].inverse()* crossproduct(temp_point_a, info.normalWorld), temp_point_a)+
		 crossproduct(this->tensors[bodyB].inverse() * crossproduct(temp_point_b, info.normalWorld), temp_point_b)) * info.normalWorld;

	return denominator/nominator;
}

void RigidBodySystemSimulator::simulateWorld(float timeStep) {
	
	if (!this->isInit) { this->init(); }

	for (int i = 0; i < this->rigidBodies.size(); i++) {
		// ----TRANSLATION:
		// 
		// Euler step position
		std::get<0>(this->rigidBodies[i]) += timeStep * this->velocities[i];
		// Euler step linear velocity
		this->velocities[i] += timeStep * this->forces[i] / std::get<2>(this->rigidBodies[i]);
		this->forces[i] = 0;
		// ----ROTATION:
		//
		// Orientation
		auto temp_rot = Quat::Quaternion(0, this->angularVelocities[i].x, this->angularVelocities[i].y, this->angularVelocities[i].z) * this->orientations[i];

		this->orientations[i] += timeStep/2 *(temp_rot.x , temp_rot.y , temp_rot.z , temp_rot.w);//?
		auto a = this->orientations[i].norm();

		std::cout << "orientation: " << this->orientations[i];
		// Angular momentum
		this->angularMomentum[i] += timeStep * torques[i];
		this->torques[i] = 0;

		// Inertia tensor
		auto temp = this->orientations[i].getRotMat();
		temp.transpose();
		Mat4 inertiaTensorInv = this->orientations[i].getRotMat() * (this->tensors[i].inverse() * temp);

		// Angular velocity
		this->angularVelocities[i] = inertiaTensorInv * this->angularMomentum[i];//?

	}

	for (int i = 0; i < this->rigidBodies.size(); i++) {
		for (int j = i + 1; j < this->rigidBodies.size(); j++) {
			// calculate Mat4 & obj2World_A, Mat4 & obj2World_B
			auto obj2World_A = Mat4{};
			auto obj2World_B = Mat4{};
			auto collision = checkCollisionSAT(obj2World_A, obj2World_B);
			if (collision.isValid) {
				std::cout << "COLLISION--------------------";
				auto impulse = this->calculateImpulse(i, j, collision);
				this->velocities[i] += impulse * collision.normalWorld / std::get<2>(this->rigidBodies[i]);
				this->velocities[j] += impulse * collision.normalWorld / std::get<2>(this->rigidBodies[j]);

				this->angularMomentum[i] += crossproduct(collision.collisionPointWorld - std::get<0>(this->rigidBodies[i]), impulse * collision.normalWorld);
				this->angularMomentum[j] -= crossproduct(collision.collisionPointWorld - std::get<0>(this->rigidBodies[j]), impulse * collision.normalWorld);

				//this->angularVelocities[i] = this->tensors[] * this->angularMomentum[i]; ?

			}
		}
	}
}
