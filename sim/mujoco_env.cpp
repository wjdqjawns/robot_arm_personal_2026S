#include "mujoco_env.hpp"
#include <stdexcept>
#include <cstring>

namespace sim {

ArmEnv::ArmEnv(const std::string& xml_path)
{
    static bool plugins_loaded = false;
    if (!plugins_loaded) {
#ifdef MUJOCO_PLUGIN_DIR
        mj_loadAllPluginLibraries(MUJOCO_PLUGIN_DIR, nullptr);
#endif
        plugins_loaded = true;
    }

    char err[1000] = {};
    m_ = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
    if (!m_) throw std::runtime_error(std::string("mj_loadXML: ") + err);
    d_ = mj_makeData(m_);
}

ArmEnv::~ArmEnv()
{
    mj_deleteData(d_);
    mj_deleteModel(m_);
}

void ArmEnv::step()
{
    mj_step(m_, d_);
    // Clear per-step wrench so callers can accumulate fresh each tick.
    mju_zero(d_->xfrc_applied, 6 * m_->nbody);
}

void ArmEnv::resetToKeyframe(int key_id)
{
    mj_resetDataKeyframe(m_, d_, key_id);
    mj_forward(m_, d_);
}

arm::ArmState ArmEnv::getState(const std::vector<int>& qpos_ids,
                               const std::vector<int>& dof_ids) const
{
    int n = static_cast<int>(dof_ids.size());
    arm::ArmState s = arm::ArmState::zero(n);
    s.t = d_->time;
    for (int i = 0; i < n; ++i) {
        s.q[i]  = d_->qpos[qpos_ids[i]];
        s.dq[i] = d_->qvel[dof_ids[i]];
    }
    return s;
}

arm::VecN ArmEnv::getActuatorTorques(const std::vector<int>& dof_ids) const
{
    int n = static_cast<int>(dof_ids.size());
    arm::VecN tau(n);
    for (int i = 0; i < n; ++i)
        tau[i] = d_->qfrc_actuator[dof_ids[i]];
    return tau;
}

void ArmEnv::applyTorques(const arm::VecN& tau,
                          const std::vector<int>& actuator_ids)
{
    int n = static_cast<int>(actuator_ids.size());
    for (int i = 0; i < n; ++i)
        d_->ctrl[actuator_ids[i]] = tau[i];
}

void ArmEnv::setBodyWrench(int body_id,
                           const Eigen::Vector3d& force,
                           const Eigen::Vector3d& torque)
{
    if (body_id < 0 || body_id >= m_->nbody) return;
    double* xfrc = &d_->xfrc_applied[6 * body_id];
    xfrc[0] += force.x();
    xfrc[1] += force.y();
    xfrc[2] += force.z();
    xfrc[3] += torque.x();
    xfrc[4] += torque.y();
    xfrc[5] += torque.z();
}

arm::MatN ArmEnv::getMassMatrix() const
{
    int nv = m_->nv;
    std::vector<mjtNum> Mfull(nv * nv);
    mj_fullM(m_, Mfull.data(), d_->qM);
    arm::MatN M = arm::MatN::Zero(nv, nv);
    for (int r = 0; r < nv; ++r)
        for (int c = 0; c < nv; ++c)
            M(r, c) = Mfull[r * nv + c];
    return M;
}

int ArmEnv::bodyId(const std::string& name) const
{
    int id = mj_name2id(m_, mjOBJ_BODY, name.c_str());
    if (id < 0)
        throw std::runtime_error("ArmEnv: body not found: " + name);
    return id;
}

int ArmEnv::jointId(const std::string& name) const
{
    int id = mj_name2id(m_, mjOBJ_JOINT, name.c_str());
    if (id < 0)
        throw std::runtime_error("ArmEnv: joint not found: " + name);
    return id;
}

int ArmEnv::dofId(int joint_id) const
{
    return m_->jnt_dofadr[joint_id];
}

int ArmEnv::actuatorId(const std::string& name) const
{
    int id = mj_name2id(m_, mjOBJ_ACTUATOR, name.c_str());
    if (id < 0)
        throw std::runtime_error("ArmEnv: actuator not found: " + name);
    return id;
}

} // namespace sim
