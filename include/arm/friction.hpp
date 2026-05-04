#pragma once
#include "types.hpp"

namespace arm {

struct FrictionConfig {
    bool enabled{false};
    VecN viscous;   // per-joint viscous coefficient Bv [Nm/(rad/s)]
    VecN coulomb;   // per-joint Coulomb friction magnitude Bc [Nm]
};

// Implements: tau_friction = viscous*dq + coulomb*sign(dq)
// In the simulation this represents unknown friction that the controller does NOT model.
// The DOB / momentum observer is expected to estimate and reject it.
class FrictionModel {
public:
    explicit FrictionModel(const FrictionConfig& cfg);

    VecN compute(const VecN& dq) const;

private:
    FrictionConfig cfg_;
};

} // namespace arm
