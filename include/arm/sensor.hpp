#pragma once
#include "types.hpp"
#include <deque>
#include <random>

namespace arm {

struct SensorConfig {
    bool   enabled{true};
    VecN   pos_noise_std;    // per-joint encoder noise [rad]
    VecN   vel_noise_std;    // per-joint velocity noise [rad/s]
    VecN   tau_noise_std;    // per-joint torque sensor noise [Nm]
    VecN   pos_bias;         // constant position bias [rad]
    VecN   vel_bias;         // constant velocity bias [rad/s]
    double drift_rate{0.0};  // position bias random-walk std per sqrt(s) [rad/sqrt(s)]
    int    delay_steps{0};   // integer sample delay (0 = no delay)
};

// Models realistic joint encoder + torque sensor imperfections.
class JointSensor {
public:
    explicit JointSensor(const SensorConfig& cfg, int n_joints);

    // Returns a noisy, potentially delayed measurement of true_state.
    ArmState read(const ArmState& true_state, double dt);

    void reset(int n);

private:
    SensorConfig cfg_;
    int          n_;
    VecN         drift_;     // accumulated random-walk bias [rad]

    std::default_random_engine       rng_{1337};
    std::normal_distribution<double> norm_{0.0, 1.0};

    std::deque<ArmState> delay_buf_;
};

} // namespace arm
