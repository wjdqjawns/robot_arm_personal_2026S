#include "arm/observer/momentum_observer.hpp"

namespace arm {

MomentumObserver::MomentumObserver(const MomentumObserverConfig& cfg)
    : cfg_(cfg)
{
    enabled_ = cfg_.enabled;
}

void MomentumObserver::reset(int n)
{
    p_hat_       = VecN::Zero(n);
    r_           = VecN::Zero(n);
    initialized_ = false;
}

void MomentumObserver::update(const ArmState& state,
                              const VecN& tau_cmd,
                              const MatN& M_q,
                              double dt)
{
    if (!enabled_) {
        r_ = VecN::Zero(state.q.size());
        return;
    }

    int n = static_cast<int>(state.q.size());

    // Extract arm submatrix of M_q (first n rows/cols, as arm joints are DOFs 0..n-1).
    MatN M_arm = M_q.topLeftCorner(n, n);

    // Measured generalized momentum.
    VecN p_meas = M_arm * state.dq;

    if (!initialized_) {
        reset(n);
        p_hat_ = p_meas;
        initialized_ = true;
        return;
    }

    // Residual: difference between measured and predicted momentum.
    VecN Ko = cfg_.Ko;
    if (Ko.size() != n) Ko = VecN::Constant(n, (Ko.size() > 0) ? Ko[0] : 10.0);

    r_ = Ko.cwiseProduct(p_meas - p_hat_);

    // Propagate estimated momentum: dp_hat/dt = tau_cmd + r
    // (Coriolis / gravity terms omitted; gravcomp="1" handles gravity internally).
    VecN tau_n = tau_cmd.head(n);
    p_hat_ += dt * (tau_n + r_);
}

} // namespace arm
