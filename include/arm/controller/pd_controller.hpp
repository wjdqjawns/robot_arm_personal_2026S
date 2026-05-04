#pragma once
#include "arm/types.hpp"

namespace arm
{
    struct PDConfig
    {
        bool   enabled{true};
        VecN   Kp;                     // per-joint proportional gain [Nm/rad]
        VecN   Kd;                     // per-joint derivative gain [Nm/(rad/s)]
        VecN   tau_max;                // per-joint torque saturation [Nm]
        bool   use_observer{false};    // subtract observer estimate from cmd
        double observer_gain{1.0};     // feedforward scale factor for d_hat
    };

    //   tau = Kp*(q_des - q) + Kd*(dq_des - dq) - observer_gain * d_hat
    class PDController
    {
    public:
        explicit PDController(const PDConfig& cfg);
        VecN compute(const ArmState& state, const VecN& q_des, const VecN& dq_des, const VecN& d_hat) const;
    private:
        PDConfig cfg_;
    };
} // namespace arm
