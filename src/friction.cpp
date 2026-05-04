#include "arm/friction.hpp"
#include <cmath>

namespace arm {

FrictionModel::FrictionModel(const FrictionConfig& cfg)
    : cfg_(cfg)
{}

VecN FrictionModel::compute(const VecN& dq) const
{
    int n = static_cast<int>(dq.size());
    VecN tau = VecN::Zero(n);

    if (!cfg_.enabled) return tau;

    for (int i = 0; i < n; ++i) {
        double bv = (i < static_cast<int>(cfg_.viscous.size()))  ? cfg_.viscous[i]  : 0.0;
        double bc = (i < static_cast<int>(cfg_.coulomb.size()))  ? cfg_.coulomb[i]  : 0.0;

        // Smooth Coulomb approximation avoids sign-function discontinuity.
        // tanh(scale * dq) → sign(dq) for large scale.
        constexpr double kSmooth = 100.0;
        tau[i] = bv * dq[i] + bc * std::tanh(kSmooth * dq[i]);
    }
    return tau;
}

} // namespace arm
