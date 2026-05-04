#include "arm/observer/dob.hpp"
#include <cmath>

namespace arm {

DOB::DOB(const DOBConfig& cfg)
    : cfg_(cfg)
{
    enabled_ = cfg_.enabled;
}

void DOB::reset(int n)
{
    d_hat_       = VecN::Zero(n);
    dq_prev_     = VecN::Zero(n);
    initialized_ = false;
}

void DOB::update(const ArmState& state,
                 const VecN& /*tau_cmd*/,
                 const MatN& /*M_q*/,
                 double dt)
{
    if (!enabled_) {
        d_hat_ = VecN::Zero(state.q.size());
        return;
    }

    int n = static_cast<int>(state.q.size());

    if (!initialized_) {
        reset(n);
        dq_prev_ = state.dq;
        initialized_ = true;
        return;
    }

    // Q-filter coefficient.
    double alpha = std::exp(-2.0 * M_PI * cfg_.cutoff_hz * dt);

    VecN residual(n);
    for (int i = 0; i < n; ++i) {
        // Finite-difference joint acceleration estimate.
        double ddq_est = (state.dq[i] - dq_prev_[i]) / dt;

        // Nominal model prediction: M_n * ddq + Bv_n * dq.
        double In = (i < static_cast<int>(cfg_.inertia_nominal.size()))
                        ? cfg_.inertia_nominal[i]
                        : 1e-3;
        double Bn = (i < static_cast<int>(cfg_.damping_nominal.size()))
                        ? cfg_.damping_nominal[i]
                        : 0.0;
        double tau_nominal = In * ddq_est + Bn * state.dq[i];

        // Residual = measured torque − nominal prediction.
        // tau_meas (state.tau_meas) is the actuator torque read from simulation.
        residual[i] = state.tau_meas[i] - tau_nominal;
    }

    // Apply Q-filter.
    d_hat_ = alpha * d_hat_ + (1.0 - alpha) * residual;
    dq_prev_ = state.dq;
}

} // namespace arm
