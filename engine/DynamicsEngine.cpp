#include "DynamicsEngine.hpp"
#include "dynamics/Gravity.hpp"
#include "frames/FrameTransformer.hpp"
#include <utility>

DynamicsEngine::DynamicsEngine(RigidBody chaser, RigidBody target, std::unique_ptr<IIntegrator> integrator)
    : m_chaser(std::move(chaser)), m_target(std::move(target)), m_integrator(std::move(integrator)) {
}

void DynamicsEngine::AddDisturbance(BodySelector who, std::unique_ptr<IDisturbanceModel> model) {
    if (who == BodySelector::Chaser) m_chaserDisturbances.push_back(std::move(model));
    else m_targetDisturbances.push_back(std::move(model));
}

void DynamicsEngine::EnableDisturbances(bool enabled) {
    m_disturbancesEnabled = enabled;
}

void DynamicsEngine::SetDockingRoleAssignment(DockingRoleAssignment role) {
    m_role = role;
}

void DynamicsEngine::Step(double dt) {
    m_chaser.ClearAccumulators();
    m_target.ClearAccumulators();
    
    // 1. Two-body gravity
    Vec3 g_chaser = TwoBodyGravity::AccelerationECI(m_chaser.positionECI);
    Vec3 g_target = TwoBodyGravity::AccelerationECI(m_target.positionECI);
    
    m_chaser.ApplyForceECI(g_chaser * m_chaser.mass);
    m_target.ApplyForceECI(g_target * m_target.mass);
    
    // 2. Disturbances (Phase 3)
    if (m_disturbancesEnabled) {
        SimState state = GetSnapshot();
        for (const auto& model : m_chaserDisturbances) {
            auto ft = model->Compute(m_chaser, state);
            m_chaser.ApplyForceECI(ft.forceECI);
            m_chaser.ApplyTorqueBody(ft.torqueBody);
        }
        for (const auto& model : m_targetDisturbances) {
            auto ft = model->Compute(m_target, state);
            m_target.ApplyForceECI(ft.forceECI);
            m_target.ApplyTorqueBody(ft.torqueBody);
        }
    }
    
    // 3. Contact forces (Phase 4)
    SimState currentState = GetSnapshot();
    m_contact.ApplyContactForces(m_chaser, m_target, currentState.relativeDockState);
    
    // 4. NGC (Always zero per SOW/plan)
    ForcesTorques extChaser{Vec3(0,0,0), Vec3(0,0,0)};
    ForcesTorques extTarget{Vec3(0,0,0), Vec3(0,0,0)};
    
    // 5. Integrate
    m_integrator->Step(m_chaser, extChaser, dt);
    m_integrator->Step(m_target, extTarget, dt);
    
    m_currentTimeS += dt;
}

SimState DynamicsEngine::GetSnapshot() const {
    SimState s;
    s.timestampS = m_currentTimeS;
    
    s.chaserECI.position = m_chaser.positionECI;
    s.chaserECI.velocity = m_chaser.velocityECI;
    s.chaserECI.attitude = m_chaser.attitude;
    s.chaserECI.angularVelocity = m_chaser.angularVelocityBody;
    
    s.targetECI.position = m_target.positionECI;
    s.targetECI.velocity = m_target.velocityECI;
    s.targetECI.attitude = m_target.attitude;
    s.targetECI.angularVelocity = m_target.angularVelocityBody;
    
    s.chaserForceECI = m_chaser.forceAccumECI;
    s.targetForceECI = m_target.forceAccumECI;
    
    // Calculate 14-state relative Y vector (Chaser Dock w.r.t Target Dock)
    DockFrame chaserDock(true, m_chaser.dockFrameOffsetBody);
    DockFrame targetDock(false, m_target.dockFrameOffsetBody);
    
    s.relativeDockState = FrameTransformer::Transform(chaserDock, targetDock, s);
    
    return s;
}
