#include "arm/sensor.hpp"
#include <cmath>

namespace arm {

JointSensor::JointSensor(const SensorConfig& cfg, int n_joints)
    : cfg_(cfg), n_(n_joints)
{
    reset(n_joints);
}

void JointSensor::reset(int n)
{
    n_     = n;
    drift_ = VecN::Zero(n);
    delay_buf_.clear();
}

ArmState JointSensor::read(const ArmState& true_state, double dt)
{
    if (!cfg_.enabled) return true_state;

    int n = static_cast<int>(true_state.q.size());
    ArmState noisy = true_state;

    // Update drift (random walk).
    if (cfg_.drift_rate > 0.0) {
        double dw_std = cfg_.drift_rate * std::sqrt(dt);
        for (int i = 0; i < n; ++i)
            drift_[i] += dw_std * norm_(rng_);
    }

    for (int i = 0; i < n; ++i) {
        // Position: bias + drift + white noise.
        double pos_bias = (i < static_cast<int>(cfg_.pos_bias.size()))
                              ? cfg_.pos_bias[i]
                              : 0.0;
        double pos_std  = (i < static_cast<int>(cfg_.pos_noise_std.size()))
                              ? cfg_.pos_noise_std[i]
                              : 0.0;
        noisy.q[i] += pos_bias + drift_[i] + pos_std * norm_(rng_);

        // Velocity: bias + white noise.
        double vel_bias = (i < static_cast<int>(cfg_.vel_bias.size()))
                              ? cfg_.vel_bias[i]
                              : 0.0;
        double vel_std  = (i < static_cast<int>(cfg_.vel_noise_std.size()))
                              ? cfg_.vel_noise_std[i]
                              : 0.0;
        noisy.dq[i] += vel_bias + vel_std * norm_(rng_);

        // Torque sensor noise.
        double tau_std = (i < static_cast<int>(cfg_.tau_noise_std.size()))
                             ? cfg_.tau_noise_std[i]
                             : 0.0;
        if (tau_std > 0.0 && i < static_cast<int>(noisy.tau_meas.size()))
            noisy.tau_meas[i] += tau_std * norm_(rng_);
    }

    // Measurement delay: push current into buffer, return oldest.
    if (cfg_.delay_steps <= 0) return noisy;

    delay_buf_.push_back(noisy);
    if (static_cast<int>(delay_buf_.size()) <= cfg_.delay_steps)
        return delay_buf_.front();  // pad with oldest until buffer full

    ArmState delayed = delay_buf_.front();
    delay_buf_.pop_front();
    return delayed;
}

} // namespace arm
