#include "arm/observer/force_observer.hpp"
#include <cmath>

namespace arm {

ForceObserver::ForceObserver(const ForceObserverConfig& cfg)
    : cfg_(cfg)
{
    enabled_ = cfg_.enabled;
}

void ForceObserver::reset(int n)
{
    d_hat_ = VecN::Zero(n);
}

void ForceObserver::update(const ArmState& state,
                           const VecN& tau_cmd,
                           const MatN& /*M_q*/,
                           double dt)
{
    if (!enabled_) {
        d_hat_ = VecN::Zero(state.q.size());
        return;
    }

    int n = static_cast<int>(state.q.size());

    // First-order low-pass filter: alpha = exp(-2*pi*fc*dt).
    double alpha = std::exp(-2.0 * M_PI * cfg_.cutoff_hz * dt);

    // Residual: what was actually applied minus what the controller commanded.
    VecN residual = state.tau_meas.head(n) - tau_cmd.head(n);

    d_hat_ = alpha * d_hat_ + (1.0 - alpha) * residual;
}

} // namespace arm
