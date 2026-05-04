#pragma once
#include <Eigen/Dense>
#include <vector>

namespace arm {

using VecN = Eigen::VectorXd;
using MatN = Eigen::MatrixXd;

// Joint-space state of the arm.
struct ArmState {
    VecN   q;         // joint positions [rad]
    VecN   dq;        // joint velocities [rad/s]
    VecN   tau_meas;  // measured joint torques (actuator output + noise) [Nm]
    double t{0.0};    // simulation time [s]

    static ArmState zero(int n) {
        ArmState s;
        s.q        = VecN::Zero(n);
        s.dq       = VecN::Zero(n);
        s.tau_meas = VecN::Zero(n);
        return s;
    }
};

// Static configuration read once from YAML at startup.
struct ArmConfig {
    int n_joints{6};
    double dt{0.002};

    // MuJoCo identifiers resolved at init time from joint/actuator names.
    std::vector<int> qpos_ids;      // m->jnt_qposadr[jid] — qpos row per joint
    std::vector<int> dof_ids;       // m->jnt_dofadr[jid]  — qvel / qfrc row per joint
    std::vector<int> actuator_ids;  // ctrl[] indices for arm actuators
};

} // namespace arm
