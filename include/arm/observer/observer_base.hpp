#pragma once
#include "arm/types.hpp"

namespace arm {

// Base interface for all disturbance observers.
// Each observer estimates the disturbance torque tau_d acting on the joints.
class ObserverBase {
public:
    virtual ~ObserverBase() = default;

    // Update internal state with current measured state, last commanded torque,
    // and the current joint-space mass matrix M(q).
    virtual void update(const ArmState& state,
                        const VecN&     tau_cmd,
                        const MatN&     M_q,
                        double          dt) = 0;

    // Returns the current disturbance torque estimate [Nm].
    virtual VecN estimate() const = 0;

    virtual void reset(int n) = 0;

    bool isEnabled() const   { return enabled_; }
    void setEnabled(bool en) { enabled_ = en; }

protected:
    bool enabled_{true};
};

} // namespace arm
