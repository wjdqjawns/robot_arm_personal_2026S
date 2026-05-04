#include "arm/controller/pd_controller.hpp"
#include <algorithm>

namespace arm {

PDController::PDController(const PDConfig& cfg)
    : cfg_(cfg)
{}

VecN PDController::compute(const ArmState& state,
                           const VecN& q_des,
                           const VecN& dq_des,
                           const VecN& d_hat) const
{
    int n = static_cast<int>(state.q.size());
    VecN tau = VecN::Zero(n);

    if (!cfg_.enabled) return tau;

    for (int i = 0; i < n; ++i) {
        double kp  = (i < static_cast<int>(cfg_.Kp.size()))      ? cfg_.Kp[i]      : 0.0;
        double kd  = (i < static_cast<int>(cfg_.Kd.size()))      ? cfg_.Kd[i]      : 0.0;
        double lim = (i < static_cast<int>(cfg_.tau_max.size())) ? cfg_.tau_max[i] : 1e9;

        double q_e  = q_des[i]  - state.q[i];
        double dq_e = dq_des[i] - state.dq[i];

        tau[i] = kp * q_e + kd * dq_e;

        // Observer feedforward: subtract disturbance estimate to cancel it at the input.
        if (cfg_.use_observer && i < static_cast<int>(d_hat.size()))
            tau[i] -= cfg_.observer_gain * d_hat[i];

        tau[i] = std::clamp(tau[i], -lim, lim);
    }
    return tau;
}

} // namespace arm
