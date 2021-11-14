#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator():
    m_fMass(10.0f),
    m_fStiffness(40.0f),
    m_fDamping(0.0f),
    m_iIntegrator(EULER)
{
    
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
    return "Test Cases...";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
}

void MassSpringSystemSimulator::reset()
{
    m_mouse.x = 0;
    m_mouse.y = 0;

    m_trackmouse.x = 0;
    m_trackmouse.y = 0;

    m_oldtrackmouse.x = 0;
    m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    switch (m_iTestCase) {
    case EULER:
        std::cout << "Euler";
        break;
    case MIDPOINT:
        std::cout << "Midpoint";
        break;
    case LEAPFROG:
        std::cout << "Leap-Frog";
        break;
    default:
        std::cout << "Unknown Test Case";
        break;
    }

    std::cout << std::endl;
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
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

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
    masspoints.emplace_back(position, Velocity, isFixed);

}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
    return 0;
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
    return 0;
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
    return Vec3();
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
    return Vec3();
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
}
