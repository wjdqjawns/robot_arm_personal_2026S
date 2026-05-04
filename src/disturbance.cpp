#include "arm/disturbance.hpp"
#include <cmath>
#include <stdexcept>

namespace arm {

DisturbanceManager::DisturbanceManager(const MatchedConfig& mc,
                                       const MismatchedConfig& mmc)
    : mc_(mc), mmc_(mmc)
{}

void DisturbanceManager::saveNominalState(mjModel* m)
{
    if (nominal_saved_) return;

    nominal_mass_.resize(m->nbody);
    for (int i = 0; i < m->nbody; ++i)
        nominal_mass_[i] = m->body_mass[i];

    nominal_gravity_ = std::abs(m->opt.gravity[2]);

    if (!mmc_.force_body.empty()) {
        force_body_id_ = mj_name2id(m, mjOBJ_BODY, mmc_.force_body.c_str());
        if (force_body_id_ < 0)
            throw std::runtime_error("DisturbanceManager: body not found: "
                                     + mmc_.force_body);
    }

    nominal_saved_ = true;
}

VecN DisturbanceManager::applyMatched(const VecN& tau_ctrl, double t)
{
    if (!mc_.enabled) return tau_ctrl;

    VecN tau = tau_ctrl;
    int  n   = static_cast<int>(tau.size());

    for (int i = 0; i < n; ++i) {
        // Constant bias.
        if (i < static_cast<int>(mc_.constant.size()))
            tau[i] += mc_.constant[i];

        // Gaussian noise.
        if (i < static_cast<int>(mc_.noise_std.size()) && mc_.noise_std[i] > 0.0)
            tau[i] += mc_.noise_std[i] * norm_(rng_);
    }

    // Chirp signal (applies to all joints uniformly, so it can be used to
    // identify matched-disturbance responses across joints simultaneously).
    if (mc_.chirp_amp > 0.0) {
        double chirp = mc_.chirp_amp
                       * std::sin(2.0 * M_PI * mc_.chirp_freq_hz * t);
        tau += VecN::Constant(n, chirp);
    }

    return tau;
}

void DisturbanceManager::applyMismatched(mjModel* m, mjData* d) const
{
    if (!mmc_.enabled) return;

    // Mass scaling (persists: set every step so it cannot drift).
    if (mmc_.mass_scale != 1.0 && nominal_saved_) {
        for (int i = 0; i < m->nbody; ++i)
            m->body_mass[i] = nominal_mass_[i] * mmc_.mass_scale;
    }

    // Gravity scaling.
    if (mmc_.gravity_scale != 1.0 && nominal_saved_) {
        m->opt.gravity[2] = -nominal_gravity_ * mmc_.gravity_scale;
    }

    // External force on a body (world frame, accumulates into xfrc_applied).
    if (force_body_id_ >= 0 &&
        mmc_.ext_force.norm() > 0.0) {
        double* xfrc = &d->xfrc_applied[6 * force_body_id_];
        xfrc[0] += mmc_.ext_force.x();
        xfrc[1] += mmc_.ext_force.y();
        xfrc[2] += mmc_.ext_force.z();
    }
}

} // namespace arm
