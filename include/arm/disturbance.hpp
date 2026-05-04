#pragma once
#include "types.hpp"
#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <random>
#include <string>
#include <vector>

namespace arm {

// ── Matched disturbance ──────────────────────────────────────────────────────
// Injected into the control input channel:  tau_effective = tau_ctrl + d_matched
struct MatchedConfig {
    bool   enabled{false};
    VecN   constant;          // per-joint constant offset [Nm]
    VecN   noise_std;         // per-joint Gaussian noise std [Nm]
    double chirp_amp{0.0};    // chirp amplitude [Nm], applied to all joints
    double chirp_freq_hz{0.5};
};

// ── Mismatched disturbance ───────────────────────────────────────────────────
// Applied to MuJoCo physics directly (not through the input channel).
struct MismatchedConfig {
    bool            enabled{false};
    double          mass_scale{1.0};      // multiplicative factor on all body masses
    std::string     force_body;           // MuJoCo body name to apply external force
    Eigen::Vector3d ext_force{0, 0, 0};  // force in world frame [N]
    double          gravity_scale{1.0};  // scale factor for gravity magnitude
};

// ── Manager ──────────────────────────────────────────────────────────────────
class DisturbanceManager {
public:
    DisturbanceManager(const MatchedConfig& mc, const MismatchedConfig& mmc);

    // Call once after model load to save nominal masses for mass_scale.
    void saveNominalState(mjModel* m);

    // Returns tau_ctrl + matched disturbance (noise, constant, chirp).
    VecN applyMatched(const VecN& tau_ctrl, double t);

    // Sets xfrc_applied / modifies m->body_mass / m->opt.gravity.
    // Must be called every step before mj_step().
    void applyMismatched(mjModel* m, mjData* d) const;

private:
    MatchedConfig    mc_;
    MismatchedConfig mmc_;

    std::default_random_engine        rng_{42};
    std::normal_distribution<double>  norm_{0.0, 1.0};

    std::vector<double> nominal_mass_;
    double              nominal_gravity_{9.81};
    int                 force_body_id_{-1};
    bool                nominal_saved_{false};
};

} // namespace arm
