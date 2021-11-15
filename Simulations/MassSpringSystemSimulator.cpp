#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator(): m_fMass(10), m_fStiffness(40), m_fDamping(0), m_fRestLength(1), m_bGravityOn(false), m_vGravity(0,-10,0), m_iIntegrator(EULER){
    
}

