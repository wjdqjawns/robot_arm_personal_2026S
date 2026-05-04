#include "mujoco_env.hpp"
#include <stdexcept>
#include <algorithm>
#include <string>

namespace sim
{
    MujocoEnv::MujocoEnv(const std::string& xml_path)
    {
        // Mesh decoders (OBJ, STL) live in separate plugin .so files in MuJoCo 3.x.
        // They must be loaded before mj_loadXML; calling this repeatedly is safe.
        static bool plugins_loaded = false;
        if (!plugins_loaded)
        {
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

    MujocoEnv::~MujocoEnv()
    {
        mj_deleteData(d_);
        mj_deleteModel(m_);
    }

    void MujocoEnv::step()
    {
        mj_step(m_, d_);
    }

    void MujocoEnv::resetToKeyframe(int key_id)
    {
        mj_resetDataKeyframe(m_, d_, key_id);
        mj_forward(m_, d_);
    }

    void MujocoEnv::setActuators(const drone::Vec4& f)
    {
        for (int i = 0; i < 4; ++i)
        {
            d_->ctrl[i] = std::clamp(f[i], 0.0, 13.0);
        }
    }

    drone::Vec3 MujocoEnv::getPosition() const
    {
        return {d_->qpos[0], d_->qpos[1], d_->qpos[2]};
    }

    drone::Vec3 MujocoEnv::getVelocity() const
    {
        return {d_->qvel[0], d_->qvel[1], d_->qvel[2]};
    }

    drone::Quat MujocoEnv::getOrientation() const
    {
        return drone::Quat(d_->qpos[3], d_->qpos[4], d_->qpos[5], d_->qpos[6]);
    }

    drone::Vec3 MujocoEnv::getAngularVelocity() const
    {
        return {d_->qvel[3], d_->qvel[4], d_->qvel[5]};
    }

    drone::Vec3 MujocoEnv::getSensorGyro() const
    {
        return {d_->sensordata[0], d_->sensordata[1], d_->sensordata[2]};
    }

    drone::Vec3 MujocoEnv::getSensorAccel() const
    {
        return {d_->sensordata[3], d_->sensordata[4], d_->sensordata[5]};
    }

    drone::Quat MujocoEnv::getSensorQuat() const
    {
        return drone::Quat(d_->sensordata[6], d_->sensordata[7], d_->sensordata[8], d_->sensordata[9]);
    }

    // ── Generic joint-space methods ───────────────────────────────────────────

    void MujocoEnv::reset()
    {
        mj_resetData(m_, d_);
        mj_forward(m_, d_);
    }

    Eigen::VectorXd MujocoEnv::getJointPos(int nq, int offset) const
    {
        Eigen::VectorXd q(nq);
        for (int i = 0; i < nq; ++i)
            q[i] = d_->qpos[offset + i];
        return q;
    }

    Eigen::VectorXd MujocoEnv::getJointVel(int nv, int offset) const
    {
        Eigen::VectorXd dq(nv);
        for (int i = 0; i < nv; ++i)
            dq[i] = d_->qvel[offset + i];
        return dq;
    }

    void MujocoEnv::setControl(int index, double value)
    {
        d_->ctrl[index] = value;
    }

    void MujocoEnv::setControls(const Eigen::VectorXd& u)
    {
        const int n = std::min<int>(static_cast<int>(u.size()), m_->nu);
        for (int i = 0; i < n; ++i)
            d_->ctrl[i] = u[i];
    }

    drone::Vec3 MujocoEnv::getBodyPos(const std::string& name) const
    {
        int id = mj_name2id(m_, mjOBJ_BODY, name.c_str());
        if (id < 0) return drone::Vec3::Zero();
        return {d_->xpos[id * 3], d_->xpos[id * 3 + 1], d_->xpos[id * 3 + 2]};
    }

    drone::Quat MujocoEnv::getBodyQuat(const std::string& name) const
    {
        int id = mj_name2id(m_, mjOBJ_BODY, name.c_str());
        if (id < 0) return drone::Quat::Identity();
        // MuJoCo stores xquat as [w, x, y, z]
        return drone::Quat(d_->xquat[id * 4],     d_->xquat[id * 4 + 1],
                           d_->xquat[id * 4 + 2], d_->xquat[id * 4 + 3]);
    }
} // namespace sim