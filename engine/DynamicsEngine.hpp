#pragma once

#include "SimState.hpp"
#include "dynamics/RigidBody.hpp"
#include "dynamics/Integrator.hpp"
#include "frames/Frame.hpp"
#include <memory>
#include <vector>

#include "dynamics/disturbances/DisturbanceModel.hpp"
#include "contact/ContactSpringModel.hpp"

enum class BodySelector { Chaser, Target };
enum class DockingRoleAssignment { ChaserActive, TargetActive };

class DynamicsEngine {
public:
    DynamicsEngine(RigidBody chaser, RigidBody target, std::unique_ptr<IIntegrator> integrator);

    void AddDisturbance(BodySelector who, std::unique_ptr<IDisturbanceModel> model);
    void EnableDisturbances(bool enabled);
    void SetDockingRoleAssignment(DockingRoleAssignment role);

    void Step(double dt);
    SimState GetSnapshot() const;

    RigidBody& GetChaser() { return m_chaser; }
    RigidBody& GetTarget() { return m_target; }

private:
    RigidBody m_chaser;
    RigidBody m_target;
    std::unique_ptr<IIntegrator> m_integrator;
    ContactSpringModel m_contact;

    std::vector<std::unique_ptr<IDisturbanceModel>> m_chaserDisturbances;
    std::vector<std::unique_ptr<IDisturbanceModel>> m_targetDisturbances;

    bool m_disturbancesEnabled = true;
    DockingRoleAssignment m_role = DockingRoleAssignment::ChaserActive;
    
    double m_currentTimeS = 0.0;
};
