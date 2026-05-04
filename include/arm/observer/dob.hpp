#pragma once
#include "observer_base.hpp"

namespace arm
{
    // ── Q-filter Disturbance Observer (DOB) ──────────────────────────────────────
    //
    // Estimates disturbance torque via a first-order Q-filter applied to the
    // torque residual between measured actuator torque and a nominal model:
    //
    //   residual[k] = tau_meas[k] - M_n * ddq[k] - Bv_n * dq[k]
    //   d_hat[k]    = alpha * d_hat[k-1] + (1 - alpha) * residual[k]
    //
    // where alpha = exp(-2*pi*cutoff_hz*dt), M_n is nominal joint inertia (diagonal),
    // Bv_n is nominal viscous damping, and ddq is estimated via finite difference.
    //
    // Reference: Ohnishi 1987 / Shim & Chung 1998.

    struct DOBConfig
    {
        bool   enabled{true};
        VecN   inertia_nominal;   // diagonal approximation of joint inertia [kg·m²]
        VecN   damping_nominal;   // nominal viscous damping per joint [Nm/(rad/s)]
        double cutoff_hz{5.0};    // Q-filter cutoff frequency
    };

    class DOB : public ObserverBase
    {
    public:
        explicit DOB(const DOBConfig& cfg);

        void update(const ArmState& state,
                    const VecN&     tau_cmd,
                    const MatN&     M_q,
                    double          dt) override;

        VecN estimate() const override { return d_hat_; }
        void reset(int n)    override;

    private:
        DOBConfig cfg_;
        VecN      d_hat_;
        VecN      dq_prev_;
        bool      initialized_{false};
    };
} // namespace arm
