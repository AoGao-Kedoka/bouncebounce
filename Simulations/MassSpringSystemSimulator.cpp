#include "MASSSPRINGSYSTEMSIMULATOR.H"
#include "MASSSPRINGSYSTEMSIMULATOR.H"
#include "MASSSPRINGSYSTEMSIMULATOR.H"
#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator():
    m_fMass(10.0f),
    m_fStiffness(40.0f),
    m_fDamping(0.0f),
    m_iIntegrator(MIDPOINT),
    m_fRestLength(1.0f),
    m_vGravityValue(0,-9.8f,0),
    m_bIsGravity(false)
{
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
    return "Demo 0: For testing, \
            Demo 1: A simple one-step test, \
            Demo 2: A simple Euler simulation, \
            Demo 3: A simple Midpoint simulation, \
            Demo 4: A complex simulation -- comparing stability of Euler and Midpoint";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;

    TwRemoveVar(DUC->g_pTweakBar, "Timestep");
    TwAddVarRO(DUC->g_pTweakBar, "Timestep", TW_TYPE_FLOAT, &m_fTimeStep, "step = 0.001");
    TwAddSeparator(DUC->g_pTweakBar, "SeperatorUp", "");
    TwAddVarRW(DUC->g_pTweakBar, "Rest Length", TW_TYPE_FLOAT, &m_fRestLength, "step = 0.2");
    TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &m_bIsGravity, "");
    if (m_iTestCase == 2 || m_iTestCase == 3) {
        TwAddVarRO(DUC->g_pTweakBar, "Integrate", TW_TYPE_INT8, &m_iIntegrator, "");
    }
    else if (m_iTestCase == 4) {
        TwRemoveVar(DUC->g_pTweakBar, "Integrate");
        TwAddVarRW(DUC->g_pTweakBar, "Integrate", TW_TYPE_INT8, &m_iIntegrator, "min=0, max=2");
        TwRemoveVar(DUC->g_pTweakBar, "Timestep");
        TwAddVarRW(DUC->g_pTweakBar, "Timestep", TW_TYPE_FLOAT, &m_fTimeStep, "step = 0.001");
    }
}

void MassSpringSystemSimulator::reset()
{
    m_mouse.x = 0;
    m_mouse.y = 0;

    m_trackmouse.x = 0;
    m_trackmouse.y = 0;

    m_oldtrackmouse.x = 0;
    m_oldtrackmouse.y = 0;

    int currentNr = springs.size();
    springs.clear();
    masspoints.clear();
    this->initSprings(currentNr);
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    for (const auto& s : springs) {
        DUC->beginLine();
        DUC->drawLine(
            getPositionOfMassPoint(s.A), {0.3,0.3,0.6}, 
            getPositionOfMassPoint(s.B), { 0.3,0.3,0.6 }
        );
        DUC->endLine();
        DUC->drawSphere(this->getPositionOfMassPoint(s.A), 0.05);
        DUC->drawSphere(this->getPositionOfMassPoint(s.B), 0.05);
    }
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    switch (m_iTestCase) {
    case 0:
        std::cout << "Demo for test usage";
        break;
    case 1:
        this->initSprings(1);
        std::cout << "Demo 1: A simple one-step test";
        break;
    case 2:
        this->initSprings(1);
        m_iIntegrator = EULER;
        std::cout << "Demo 2: A simple Euler simulation";
        break;
    case 3:
        this->initSprings(1);
        m_iIntegrator = MIDPOINT;
        std::cout << "Demo 3: A simple Midpoint simulation";
        break;
    case 4:
        this->reset();
        this->initSprings(10);
        std::cout << "Demo 4: A complex simulation, comparing stability of Euler and Midpoint";
        break;
    default:
        std::cout << "Unknown Test Case";
        break;
    }

    std::cout << std::endl;
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
    // Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
    Point2D mouseDiff;
    mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
    mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
    if (mouseDiff.x != 0 || mouseDiff.y != 0)
    {
        Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
        worldViewInv = worldViewInv.inverse();
        Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
        Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
        // find a proper scale!
        float inputScale = 0.00001f;
        inputWorld = inputWorld * inputScale;
        for (int i = 0; i < masspoints.size(); i++)
            masspoints[i].p += inputWorld;
    }
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
    switch (m_iTestCase) {
    case 0:
        if (m_iIntegrator == EULER)
            this->computeEuler(timeStep);
        else if (m_iIntegrator == MIDPOINT)
            this->computeMidPoint(timeStep);
        break;
    case 1:
        m_fTimeStep = 0.1f;
        // do only 1 iteration
        if (masspoints[springs[0].A].p.x == 0) {
            this->computeEuler(m_fTimeStep);
            std::cout << "Calculated with Euler method: " << std::endl;
            this->writeState();
            this->reset();
            if (masspoints[springs[0].A].p.x == 0) {
                this->computeMidPoint(m_fTimeStep);
                std::cout << "Calculated with MidPoint method: " << std::endl;
                this->writeState();
            }
        }
        
        break;
    case 2:
        m_fTimeStep = 0.005f;
        this->computeEuler(m_fTimeStep);
        break;
    case 3:
        m_fTimeStep = 0.005f;
        this->computeMidPoint(m_fTimeStep);
        break;
    case 4:
        //m_fTimeStep = timeStep;
        if (m_iIntegrator == EULER)
            this->computeEuler(m_fTimeStep);
        else if(m_iIntegrator == MIDPOINT)
            this->computeMidPoint(m_fTimeStep);
        break;
    default:
        if (m_iIntegrator == EULER)
            this->computeEuler(timeStep);
        if (m_iIntegrator == MIDPOINT)
            this->computeMidPoint(timeStep);
        break;
    }
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
    m_trackmouse.x = x;
    m_trackmouse.y = y;

    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;

}

void MassSpringSystemSimulator::setMass(float mass)
{
    m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
    m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
    m_fDamping = damping;
}

void MassSpringSystemSimulator::setGravityValue(Vec3 g)
{
    m_vGravityValue = g;
}

void MassSpringSystemSimulator::setGravityValue(float x, float y, float z)
{
    m_vGravityValue.x = x;
    m_vGravityValue.y = y;
    m_vGravityValue.z = z;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
    masspoints.emplace_back(position, Velocity, isFixed);

    return masspoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int i, int j, float L)
{
    if (i >= masspoints.size() || j >= masspoints.size())
        throw std::invalid_argument("Masspoint index out of range.");

    springs.emplace_back(i, j, L);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
    return masspoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
    return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
    return masspoints[index].p;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
    return masspoints[index].v;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
    m_externalForce += force;
}

void MassSpringSystemSimulator::initSprings(int nr)
{
    springs.clear();
    masspoints.clear();
    if (nr == 0) return;
    if (nr == 1) {
        Vec3 p1 = { 0,0,0 };
        Vec3 p2 = { 0,2,0 };
        Vec3 v1= { -1,0,0 };
        Vec3 v2= { 1,0,0 };
        int A = addMassPoint(p1, v1, false);
        int B = addMassPoint(p2, v2, false);
        addSpring(A, B, m_fRestLength);
    }
    else {
        for (int i = 0; i < nr; i++)
            for (int j = 0; j < nr; j++)
                    addMassPoint(
                        Vec3(
                            (float)i / 2 - 0.4,
                            (float)j / 2 - 0.4,
                            0.f
                        ),
                        Vec3(0,0,0), 
                        false);

        for (int i = 0; i < masspoints.size() - nr; i++) {
            if(! (i%nr==1))addSpring(i, i + 1, m_fRestLength);
            addSpring(i, i+nr, m_fRestLength);
        }
         
    }
}

float MassSpringSystemSimulator::calcDist(Vec3 A, Vec3 B, Vec3* dir) {
    float dX = A.x - B.x;
    float dY = A.y - B.y;
    float dZ = A.z - B.z;

    dir->x = dX;
    dir->y = dY;
    dir->z = dZ;

    return std::sqrt(
        std::pow(dX, 2) + 
        std::pow(dY, 2) + 
        std::pow(dZ, 2)
    );
}

std::pair<Vec3, Vec3> MassSpringSystemSimulator::calcAcc(Vec3 A, Vec3 B) {
    Vec3 dir;

    auto distance = this->calcDist(A, B, &dir);

    Vec3 force = m_fStiffness * 
        (distance - m_fRestLength) * 
        (dir/distance);

    Vec3 a1 = -force / m_fMass;
    Vec3 a2 = force / m_fMass;

    if (m_bIsGravity) {
        a1 += m_vGravityValue;
        a2 += m_vGravityValue;
    }

    return std::make_pair(a1, a2);
}

void MassSpringSystemSimulator::computeEuler(float timeStep) {
    for (const auto& s : springs) {
        auto* A = &masspoints[s.A];
        auto* B = &masspoints[s.B];
        
        // Update position with an explicit Euler step
        A->p += timeStep * A->v;
        B->p += timeStep * B->v;

        // Update velocity with an explicit Euler step
        auto acc = this->calcAcc(A->p, B->p);
        A->v += timeStep * acc.first;
        B->v += timeStep * acc.second;

        // bounce from the floor
        if (A->p.y <= -1) A->v = -A->v;
        if (B->p.y <= -1) B->v = -B->v;
    }
}

void MassSpringSystemSimulator::computeMidPoint(float timeStep) {

    for (const auto& s : springs) {
        auto* A = &masspoints[s.A];
        auto* B = &masspoints[s.B];

        // compute midpoint
        auto midStepPos1 = A->p + 1 / 2 * timeStep * A->v;
        auto midStepPos2 = B->p + 1 / 2 * timeStep * B->v;

        auto acc = this->calcAcc(A->p, B->p);

        auto midStepVel1 = A->v + 1 / 2 * timeStep * acc.first;
        auto midStepVel2 = B->v + 1 / 2 * timeStep * acc.second;
        acc = this->calcAcc(midStepPos1, midStepPos2);

        // compute full step
        A->p += timeStep * midStepVel1;
        B->p += timeStep * midStepVel2;
        A->v += timeStep * acc.first;
        B->v += timeStep * acc.second;

        // bounce bounce from the floor
        if (A->p.y <= -1)
            A->v= -A->v;
        if (B->p.y <= -1)
            B->v= -B->v;
    }
}

// same as euler, but update velocity first
void MassSpringSystemSimulator::computeLeapFrog(float timeStep) {

    for (const auto& s : springs) {
        auto* A = &masspoints[s.A];
        auto* B = &masspoints[s.B];

        // Leap Frog --> Update velocity with an explicit Euler step
        auto acc = this->calcAcc(A->p, B->p);
        A->v += timeStep * acc.first;
        B->v += timeStep * acc.second;

        // Update position with an explicit Euler step
        A->p += timeStep * A->v;
        B->p += timeStep * B->v;

        // bounce from the floor
        if (A->p.y <= -1) A->v = -A->v;
        if (B->p.y <= -1) B->v = -B->v;
    }
}

void MassSpringSystemSimulator::writeState() {
    for (int i = 0; i < masspoints.size(); i++) {
        std::cout << "Masspoint " << i << "'s position: " << masspoints[i].p << std::endl;
        std::cout << "Masspoint " << i << "'s velocity: " << masspoints[i].v << std::endl;
    }
}