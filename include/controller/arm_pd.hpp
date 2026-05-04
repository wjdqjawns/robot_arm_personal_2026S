#pragma once
#include "robot_arm/state.hpp"
#include <Eigen/Dense>
#include <stdexcept>

namespace arm
{
    struct ArmPdParams
    {
        int             njoints = 6;
        Eigen::VectorXd Kp;       // proportional gains per joint [N·m/rad]
        Eigen::VectorXd Kd;       // derivative gains per joint   [N·m·s/rad]
        double          tau_max  = 30.0;  // symmetric per-joint saturation [N·m]

        ArmPdParams()
            : Kp(Eigen::VectorXd::Ones(6) * 100.0)
            , Kd(Eigen::VectorXd::Ones(6) * 10.0)
        {}

        explicit ArmPdParams(int n)
            : njoints(n)
            , Kp(Eigen::VectorXd::Ones(n) * 100.0)
            , Kd(Eigen::VectorXd::Ones(n) * 10.0)
        {}
    };

    class ArmPdController
    {
    public:
        explicit ArmPdController(const ArmPdParams& p);

        // tau = Kp*(q_des - q) + Kd*(dq_des - dq), clamped to ±tau_max per joint.
        Eigen::VectorXd compute(const ArmState&         state,
                                const Eigen::VectorXd&  q_des,
                                const Eigen::VectorXd&  dq_des) const;

        const ArmPdParams& params() const { return params_; }

    private:
        ArmPdParams params_;
    };
} // namespace arm
