#pragma once
#include "observer_base.hpp"

namespace arm {

// ── Force Observer ────────────────────────────────────────────────────────────
//
// The simplest observer: filters the torque residual between the measured
// actuator torque and the pure controller command, with no model assumption.
//
//   residual[k] = tau_meas[k] - tau_cmd[k]
//   d_hat[k]    = alpha * d_hat[k-1] + (1 - alpha) * residual[k]
//
// Estimates friction + matched disturbances.
// Does NOT require a nominal inertia model (unlike DOB).

struct ForceObserverConfig {
    bool   enabled{true};
    double cutoff_hz{10.0};
};

class ForceObserver : public ObserverBase {
public:
    explicit ForceObserver(const ForceObserverConfig& cfg);

    void update(const ArmState& state,
                const VecN&     tau_cmd,
                const MatN&     M_q,
                double          dt) override;

    VecN estimate() const override { return d_hat_; }
    void reset(int n)    override;

private:
    ForceObserverConfig cfg_;
    VecN                d_hat_;
};

} // namespace arm
