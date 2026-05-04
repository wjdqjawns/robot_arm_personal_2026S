#pragma once
#include "arm/types.hpp"
#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace sim {

// MuJoCo environment wrapper for the Piper arm (torque-controlled).
//
// Control flow: caller builds tau_applied, calls applyTorques(), then step().
// The mass matrix is recomputed by MuJoCo during step(); call getMassMatrix()
// after step() to get M(q) for the *new* configuration.
class ArmEnv {
public:
    explicit ArmEnv(const std::string& xml_path);
    ~ArmEnv();

    ArmEnv(const ArmEnv&)            = delete;
    ArmEnv& operator=(const ArmEnv&) = delete;

    // Advance by one timestep. Reads ctrl[], qfrc_applied, xfrc_applied.
    void step();

    // Reset to keyframe (default = "home").
    void resetToKeyframe(int key_id = 0);

    // ── State readout ────────────────────────────────────────────────────────

    // Ground-truth joint state.
    // qpos_ids: jnt_qposadr[jid] — used to read qpos.
    // dof_ids:  jnt_dofadr[jid]  — used to read qvel.
    arm::ArmState getState(const std::vector<int>& qpos_ids,
                           const std::vector<int>& dof_ids) const;

    // Measured actuator torques: qfrc_actuator at the given DOF indices.
    arm::VecN getActuatorTorques(const std::vector<int>& dof_ids) const;

    // ── Control input ────────────────────────────────────────────────────────

    // Write torques into ctrl[] at the given actuator indices.
    void applyTorques(const arm::VecN& tau,
                      const std::vector<int>& actuator_ids);

    // Apply a 6-DOF spatial force (fx,fy,fz, tx,ty,tz) on a body (world frame).
    // Accumulated in xfrc_applied; cleared to zero in step().
    void setBodyWrench(int body_id,
                       const Eigen::Vector3d& force,
                       const Eigen::Vector3d& torque = Eigen::Vector3d::Zero());

    // ── Dynamics ─────────────────────────────────────────────────────────────

    // Full n_v × n_v joint-space mass matrix M(q).
    arm::MatN getMassMatrix() const;

    // ── Lookup helpers ───────────────────────────────────────────────────────
    int bodyId(const std::string& name) const;
    int jointId(const std::string& name) const;
    int dofId(int joint_id) const;           // jnt_dofadr[joint_id]
    int actuatorId(const std::string& name) const;

    mjModel* model() const { return m_; }
    mjData*  data()  const { return d_; }
    double   time()  const { return d_->time; }

private:
    mjModel* m_{nullptr};
    mjData*  d_{nullptr};
};

} // namespace sim
