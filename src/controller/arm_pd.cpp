#include "controller/arm_pd.hpp"
#include <algorithm>

namespace arm
{
    ArmPdController::ArmPdController(const ArmPdParams& p) : params_(p)
    {
        if (p.Kp.size() != p.njoints || p.Kd.size() != p.njoints)
            throw std::invalid_argument("ArmPdParams: gain vector length != njoints");
    }

    Eigen::VectorXd ArmPdController::compute(const ArmState&        state,
                                              const Eigen::VectorXd& q_des,
                                              const Eigen::VectorXd& dq_des) const
    {
        const int n = params_.njoints;
        Eigen::VectorXd tau(n);
        for (int i = 0; i < n; ++i)
        {
            double t = params_.Kp[i] * (q_des[i]  - state.q[i])
                     + params_.Kd[i] * (dq_des[i] - state.dq[i]);
            tau[i] = std::clamp(t, -params_.tau_max, params_.tau_max);
        }
        return tau;
    }
} // namespace arm
