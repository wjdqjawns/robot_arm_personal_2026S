#pragma once
#include "robot_arm/state.hpp"
#include <Eigen/Dense>
#include <random>

namespace arm
{
    struct JointNoiseParams
    {
        double pos_noise  = 1e-4;  // position noise std dev  [rad]
        double vel_noise  = 1e-3;  // velocity noise std dev  [rad/s]
        double tau_noise  = 0.01;  // torque disturbance std dev [N·m]
    };

    // Additive Gaussian noise on joint position, velocity, and torque.
    class JointNoise
    {
    public:
        explicit JointNoise(const JointNoiseParams& p, unsigned seed = 42)
            : params_(p), gen_(seed), dist_(0.0, 1.0)
        {}

        // Returns a copy of state with white noise added to q and dq.
        ArmState addSensorNoise(const ArmState& s) const
        {
            ArmState out(static_cast<int>(s.q.size()));
            out.ee = s.ee;
            for (int i = 0; i < s.q.size(); ++i)
            {
                out.q[i]  = s.q[i]  + params_.pos_noise * dist_(gen_);
                out.dq[i] = s.dq[i] + params_.vel_noise * dist_(gen_);
            }
            return out;
        }

        // Returns tau with an independent disturbance added per joint.
        Eigen::VectorXd disturbTorque(const Eigen::VectorXd& tau) const
        {
            Eigen::VectorXd out = tau;
            for (int i = 0; i < tau.size(); ++i)
                out[i] += params_.tau_noise * dist_(gen_);
            return out;
        }

    private:
        JointNoiseParams                         params_;
        mutable std::mt19937                     gen_;
        mutable std::normal_distribution<double> dist_;
    };
} // namespace arm
