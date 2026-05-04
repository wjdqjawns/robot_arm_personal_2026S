#pragma once
#include "observer_base.hpp"

namespace arm {

// ── Momentum Observer (De Luca & Mattone, 2003) ──────────────────────────────
//
// Tracks generalized momentum p = M(q)*dq and computes a residual signal r
// that converges to the disturbance torque.
//
//   p[k]     = M(q[k]) * dq[k]          (use MuJoCo mass matrix)
//   r[k]     = Ko * (p[k] - p_hat[k])
//   p_hat[k+1] = p_hat[k] + dt * (tau_cmd[k] + r[k])
//
// The term C(q,dq)*dq is omitted here; it can be included by passing a
// non-zero beta correction via the extended interface.
//
// Converges to: r -> tau_friction + tau_matched_disturbance + (M - M_n)*ddq

struct MomentumObserverConfig {
    bool enabled{true};
    VecN Ko;   // per-joint observer gain [1/s]
};

class MomentumObserver : public ObserverBase {
public:
    explicit MomentumObserver(const MomentumObserverConfig& cfg);

    void update(const ArmState& state,
                const VecN&     tau_cmd,
                const MatN&     M_q,
                double          dt) override;

    VecN estimate() const override { return r_; }
    void reset(int n)    override;

private:
    MomentumObserverConfig cfg_;
    VecN p_hat_;       // estimated momentum M*dq
    VecN r_;           // residual ≈ disturbance torque
    bool initialized_{false};
};

} // namespace arm
