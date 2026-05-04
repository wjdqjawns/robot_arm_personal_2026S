#pragma once
#include <Eigen/Dense>
#include <optional>

namespace arm
{
    struct ArmState
    {
        Eigen::VectorXd q;   // joint positions  [rad]
        Eigen::VectorXd dq;  // joint velocities [rad/s]

        struct EEPose
        {
            Eigen::Vector3d    pos;
            Eigen::Quaterniond quat;
        };
        std::optional<EEPose> ee;

        ArmState() = default;
        explicit ArmState(int njoints)
            : q(Eigen::VectorXd::Zero(njoints))
            , dq(Eigen::VectorXd::Zero(njoints))
        {}
    };
} // namespace arm
