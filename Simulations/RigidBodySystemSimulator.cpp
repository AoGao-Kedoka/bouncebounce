#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_fGravity = Vec3(0,-10.,0);
    // Gravity Multiplier for UI interaction
    m_fGravityMult = 1.0f;

	m_externalForce = Vec3(0, 0, 0);


	// UI Attributes
	Point2D m_mouse = Point2D();
	Point2D m_trackmouse = Point2D();
	Point2D m_oldtrackmouse = Point2D();
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1: One-step simple test,\
            Demo 2: Single simple body,\
            Demo 3: Two rigid bodies colliding\
            Demo 4: Multiple bodies";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	
	TwAddVarRW(DUC->g_pTweakBar, "Gravity Multiplier", TW_TYPE_FLOAT, &m_fGravityMult, "min=0.00 max=3.00 step=0.1");
}

void RigidBodySystemSimulator::reset()
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	Boxes.clear();
	forces.clear();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    this->DUC->setUpLighting(Vec3(0, 0, 0),
        0.3 * Vec3(1, 1, 1),
        1500.0,
        Vec3(0.5, 0.5, 0.5));

	for (RigidBodyBox& b : Boxes) {
		Mat4 scalingMatrix = Mat4(0.0);

		scalingMatrix.initScaling(b.width, b.depth, b.height);

		Mat4 translationMatrix = Mat4(0.0);
		translationMatrix.initTranslation(b.position.x, b.position.y, b.position.z);

		Mat4 rotationMatrix = b.orientation.getRotMat();

		b.worldMatrix = scalingMatrix * rotationMatrix * translationMatrix;

		DUC->drawRigidBody(b.worldMatrix);
	}

}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
    this->reset();
    switch (testCase) {
    // Single box falling over as seen on Sample Video 1
    case 0:
        m_fGravity = Vec3(0., 0., 0.);

        // addRigidBody(position, size, mass)
        addRigidBody(Vec3(0.0, 0.2, 0.0), Vec3(.7, 1.5, .7), 2);
        
        //forcePushingOnCorner
        Boxes[0].angularMomentum = Vec3(1., 1., 1.);
        Boxes[0].orientation = Quat(Vec3(0, 1, 1), M_PI);
        Boxes[0].linearVelocity = Vec3(0, 0, 0);
        
        break;
    
    case 1:
        m_fGravity = Vec3(.0, .0, .0);

        addRigidBody(Vec3(-0.7, 0.0, 0.0), Vec3(0.5, 0.5, 0.5), 2);
        Boxes[0].orientation = Quat(Vec3(0, 1, 1), M_PI_2);
        Boxes[0].linearVelocity = Vec3(0.4, 0, 0);

        addRigidBody(Vec3(0.7, 0.0, 0.0), Vec3(0.5, 0.5, 0.5), 2);
        Boxes[1].orientation = Quat(Vec3(0, 1, 0), M_PI_2/4);
        Boxes[1].linearVelocity = Vec3(-0.4, 0, 0);
        
        break;

    case 2:
        m_fGravity = Vec3(0, -9.0f, 0);

        addRigidBody(Vec3(-1., 0.0, 0.0), Vec3(0.5, 0.5, 0.5), 1);
        Boxes[0].orientation = Quat(Vec3(0, 1, 1), M_PI_2);
        Boxes[0].linearVelocity = Vec3(-0.1, 0, 0);
        addRigidBody(Vec3(0.7, 0.0, 0.0), Vec3(0.5, 0.5, 0.5), 1);
        Boxes[1].orientation = Quat(Vec3(0, 1, 0), -M_PI_2);
        Boxes[1].linearVelocity = Vec3(-0.1, 0, 0);
        addRigidBody(Vec3(0.7, 1., 0.0), Vec3(0.5, 0.5, 0.5), 1);
        Boxes[2].orientation = Quat(Vec3(0, 1, 0), M_PI_2);
        Boxes[2].linearVelocity = Vec3(-0.1, 0, 0);
        addRigidBody(Vec3(-1., 1, 0.0), Vec3(0.5, 0.5, 0.5), 1);
        Boxes[3].orientation = Quat(Vec3(1, 1, 0), -M_PI_2);
        Boxes[3].linearVelocity = Vec3(0.1, 0, 0);
        break;
    }
}

// Apply an external force to one of the bodies
// force is applied based on mouse movement
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
    Point2D deltaMouse;
    deltaMouse.x = m_trackmouse.x - m_oldtrackmouse.x;
    deltaMouse.y = m_trackmouse.y - m_oldtrackmouse.y;

    m_externalForce = Vec3(0, 0, 0);

    if (deltaMouse.x != 0 || deltaMouse.y != 0)
    {
        Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
        worldViewInv = worldViewInv.inverse();
        Vec3 inputView = Vec3((float)deltaMouse.x, (float)-deltaMouse.y, 0);
        Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
        
        // scale to a proper value
        float inputScale = 0.001f;
        inputWorld = inputWorld * inputScale;
        m_externalForce = inputWorld;
        Boxes[0].linearVelocity += timeElapsed * m_externalForce / Boxes[0].mass;

    }
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{

    for (force& f : forces) {
        // calculate torque q for external forces
        Vec3 q = cross(f.p, f.f);

        // integrate angular momentum L
        Boxes[f.i].angularMomentum += timeStep * q;

        Boxes[f.i].totalForce += f.f;
    }


    for (RigidBodyBox& b : this->Boxes) {
        // don't simulate fixed objects
        if (b.isFixed) continue;

        double mass = b.mass;

        // Integrate orientation r using angular velocity w
        // Quat wQuat = Quat(b.angularVelocity, M_PI_2);
        // create (0, w) == (0, w.x, w.y, w.z)
        Quat wQuat;
        wQuat.x = 0;
        wQuat.y = b.angularVelocity.x;
        wQuat.z = b.angularVelocity.y;
        wQuat.w = b.angularVelocity.z;
            
        b.orientation += timeStep / 2 * wQuat * b.orientation;
        // L2-normalize orientaton r
        // A quaternion describes only rotation (without skewing) as long as it is of unit length
        b.orientation /= sqrtf(
                            b.orientation.x * b.orientation.x + 
                            b.orientation.y * b.orientation.y +
                            b.orientation.z * b.orientation.z + 
                            b.orientation.w * b.orientation.w );

        // Update the precomputed "Covariance Matrix" 
        // Mass is distributed uniformly in the box -> calculate inertia accordingly
        // Inertia of "Cuboid" from https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        b.diagInertiaInv = Vec3(1 / (mass / 12 * (b.width * b.width + b.depth * b.depth)),
                                1 / (mass / 12 * (b.width * b.width + b.depth * b.depth)),
                                1 / (mass / 12 * (b.width * b.width + b.height * b.height))
        );

        // w(t+h) = I^(-1) * L(t+h)
        b.angularVelocity = Vec3(b.diagInertiaInv[0] * b.angularMomentum[0],
                                 b.diagInertiaInv[1] * b.angularMomentum[1],
                                 b.diagInertiaInv[2] * b.angularMomentum[2]);

        // Account for gravity (already zero if turned off)
        // gravity multiplier is for UI interaction
        b.totalForce += m_fGravity * m_fGravityMult;

        // Update position and velocity of the c.o.m.
        b.position += timeStep * b.linearVelocity;
        b.linearVelocity += timeStep * b.totalForce / mass;

        // Reset total force for the next timestep calculation
        b.totalForce = Vec3(0, 0, 0);

        // Ground floor collision
        double ground = -0.5;
        if (b.position.y < ground) {
            b.position.y = ground + 0.01;
            b.linearVelocity.y *= -1;
        }

    }

    // Check for collisions
    for (int i = 0; i < Boxes.size(); i++) {
        for (int j = i + 1; j < Boxes.size(); j++) {
            CollisionInfo collisionInfo = checkCollisionSAT(Boxes[i].worldMatrix, Boxes[j].worldMatrix);
            calcImpulse(collisionInfo, Boxes[i], Boxes[j], 0);
        }
    }
    
    forces.clear();
}

void RigidBodySystemSimulator::calcImpulse(CollisionInfo info, RigidBodyBox& rb1, RigidBodyBox& rb2, int c) {
    // nothing to do if there is no collision
    if (!info.isValid) return;
    Vec3 posA = info.collisionPointWorld - rb1.position;
    Vec3 posB = info.collisionPointWorld - rb2.position;

    //Calculate v_i for the Collision Point for both objects
    Vec3 velA = rb1.linearVelocity + cross(rb1.angularVelocity, posA);
    Vec3 velB = rb2.linearVelocity + cross(rb2.angularVelocity, posB);
    Vec3 velRel = (velA - velB);

    // return if the objects are not moving towards each other
    if (dot(velRel, info.normalWorld) >= 0) return;

    // Impulse J
    double J = 0.0;

    double INumerator = -(1 + c) * dot(velRel, info.normalWorld);

    // TODO For the next 4 calculations, we feel like we should use posA and posB
    // for local positions, but that way the simulation explodes. 
    // Using world positions seems to be fine though.
    Vec3 crossproduct1 = cross(rb1.position, info.normalWorld);
    Vec3 crossproduct2 = cross(rb2.position, info.normalWorld);

    Vec3 rb1Moving = cross(Vec3(crossproduct1[0] * rb1.diagInertiaInv[0], 
                                crossproduct1[1] * rb1.diagInertiaInv[1], 
                                crossproduct1[2] * rb1.diagInertiaInv[2]), rb1.position);

    Vec3 rb2Moving = cross(Vec3(crossproduct2[0] * rb2.diagInertiaInv[0], 
                                crossproduct2[1] * rb2.diagInertiaInv[1], 
                                crossproduct2[2] * rb2.diagInertiaInv[2]), rb2.position);

    float IDenominator = 0.0;

    if (rb1.isFixed && !rb2.isFixed) {
        IDenominator = 1/rb2.mass + dot(rb2Moving, info.normalWorld);
    }
    if (!rb1.isFixed && rb2.isFixed) {
        IDenominator = 1/rb1.mass + dot(rb1Moving, info.normalWorld);
    }
    if (!rb1.isFixed && !rb2.isFixed) {
        IDenominator = 1/rb1.mass + 1/rb2.mass + dot(rb1Moving + rb2Moving, info.normalWorld);
    }

    J = INumerator / IDenominator;

    // Update Linear Velocity
    rb1.linearVelocity += J * info.normalWorld / rb1.mass;
    rb2.linearVelocity -= J * info.normalWorld / rb2.mass;

    // Update  Angular Momentum
    rb1.angularMomentum += cross(rb1.position, (J * info.normalWorld));
    rb2.angularMomentum -= cross(rb2.position, (J * info.normalWorld));
    
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 p, Vec3 f) {
    forces.emplace_back(f, p, i);
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
	return Boxes.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return Boxes[i].position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
    return Boxes[i].linearVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
    return Boxes[i].angularVelocity;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    Boxes.emplace_back(position, size, mass);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
    Boxes[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
    Boxes[i].linearVelocity = velocity;
}
