#include "arm/types.hpp"
#include "arm/disturbance.hpp"
#include "arm/sensor.hpp"
#include "arm/friction.hpp"
#include "arm/observer/dob.hpp"
#include "arm/observer/momentum_observer.hpp"
#include "arm/observer/force_observer.hpp"
#include "arm/controller/pd_controller.hpp"

#include "mujoco_env.hpp"
#include "visualizer.hpp"

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <thread>

// ── YAML helpers ──────────────────────────────────────────────────────────────
static arm::VecN yamlVec(const YAML::Node& n, int size, double def = 0.0)
{
    arm::VecN v = arm::VecN::Constant(size, def);
    if (!n || !n.IsSequence())
    {
        return v;
    }
    
    for (int i = 0; i < std::min(size, static_cast<int>(n.size())); ++i)
    {
        v[i] = n[i].as<double>(def);
    }
    return v;
}

// ── Config loaders ────────────────────────────────────────────────────────────
static arm::ArmConfig loadArmConfig(const YAML::Node& root,
                                    sim::ArmEnv& env)
{
    arm::ArmConfig cfg;
    YAML::Node a = root["arm"];
    cfg.n_joints  = a["n_joints"].as<int>(6);
    cfg.dt        = env.model()->opt.timestep;

    // Resolve joint qpos/dof addresses and actuator IDs from names.
    if (a["joint_names"] && a["joint_names"].IsSequence())
    {
        for (const auto& jn : a["joint_names"])
        {
            int jid = env.jointId(jn.as<std::string>());
            cfg.qpos_ids.push_back(env.model()->jnt_qposadr[jid]);
            cfg.dof_ids.push_back(env.dofId(jid));
        }
    }
    if (a["actuator_names"] && a["actuator_names"].IsSequence())
    {
        for (const auto& an : a["actuator_names"])
        {
            cfg.actuator_ids.push_back(env.actuatorId(an.as<std::string>()));
        }
    }
    return cfg;
}

static arm::PDConfig loadPD(const YAML::Node& root, int n)
{
    arm::PDConfig cfg;
    YAML::Node c = root["controller"];
    cfg.Kp              = yamlVec(c["Kp"],      n, 40.0);
    cfg.Kd              = yamlVec(c["Kd"],      n,  5.0);
    cfg.tau_max         = yamlVec(c["tau_max"], n, 30.0);
    cfg.use_observer    = c["use_observer"].as<bool>(false);
    cfg.observer_gain   = c["observer_gain"].as<double>(1.0);
    return cfg;
}

static arm::SensorConfig loadSensor(const YAML::Node& root, int n)
{
    arm::SensorConfig cfg;
    YAML::Node s = root["sensor"];
    cfg.enabled        = s["enabled"].as<bool>(true);
    cfg.pos_noise_std  = yamlVec(s["pos_noise_std"], n, 0.0005);
    cfg.vel_noise_std  = yamlVec(s["vel_noise_std"], n, 0.01);
    cfg.tau_noise_std  = yamlVec(s["tau_noise_std"], n, 0.05);
    cfg.pos_bias       = yamlVec(s["pos_bias"],      n, 0.0);
    cfg.vel_bias       = yamlVec(s["vel_bias"],      n, 0.0);
    cfg.drift_rate     = s["drift_rate"].as<double>(0.0);
    cfg.delay_steps    = s["delay_steps"].as<int>(0);
    return cfg;
}

static arm::FrictionConfig loadFriction(const YAML::Node& root, int n)
{
    arm::FrictionConfig cfg;
    YAML::Node f = root["friction"];
    cfg.enabled  = f["enabled"].as<bool>(false);
    cfg.viscous  = yamlVec(f["viscous"], n, 0.0);
    cfg.coulomb  = yamlVec(f["coulomb"], n, 0.0);
    return cfg;
}

static arm::MatchedConfig loadMatched(const YAML::Node& root, int n)
{
    arm::MatchedConfig cfg;
    YAML::Node m  = root["matched_disturbance"];
    cfg.enabled   = m["enabled"].as<bool>(false);
    cfg.constant  = yamlVec(m["constant"],   n, 0.0);
    cfg.noise_std = yamlVec(m["noise_std"],  n, 0.0);
    cfg.chirp_amp     = m["chirp_amp"].as<double>(0.0);
    cfg.chirp_freq_hz = m["chirp_freq_hz"].as<double>(0.5);
    return cfg;
}

static arm::MismatchedConfig loadMismatched(const YAML::Node& root)
{
    arm::MismatchedConfig cfg;
    YAML::Node m      = root["mismatched_disturbance"];
    cfg.enabled       = m["enabled"].as<bool>(false);
    cfg.mass_scale    = m["mass_scale"].as<double>(1.0);
    cfg.force_body    = m["force_body"].as<std::string>("");
    cfg.gravity_scale = m["gravity_scale"].as<double>(1.0);
    if (m["ext_force"] && m["ext_force"].IsSequence() && m["ext_force"].size() >= 3)
        cfg.ext_force = Eigen::Vector3d(m["ext_force"][0].as<double>(), m["ext_force"][1].as<double>(), m["ext_force"][2].as<double>());
    return cfg;
}

static arm::DOBConfig loadDOB(const YAML::Node& root, int n)
{
    arm::DOBConfig cfg;
    YAML::Node d       = root["dob"];
    cfg.enabled        = d["enabled"].as<bool>(true);
    cfg.inertia_nominal = yamlVec(d["inertia_nominal"], n, 0.05);
    cfg.damping_nominal = yamlVec(d["damping_nominal"], n, 0.0);
    cfg.cutoff_hz      = d["cutoff_hz"].as<double>(5.0);
    return cfg;
}

static arm::MomentumObserverConfig loadMO(const YAML::Node& root, int n)
{
    arm::MomentumObserverConfig cfg;
    YAML::Node mo  = root["momentum_observer"];
    cfg.enabled    = mo["enabled"].as<bool>(true);
    cfg.Ko         = yamlVec(mo["Ko"], n, 20.0);
    return cfg;
}

static arm::ForceObserverConfig loadFO(const YAML::Node& root)
{
    arm::ForceObserverConfig cfg;
    YAML::Node fo  = root["force_observer"];
    cfg.enabled    = fo["enabled"].as<bool>(true);
    cfg.cutoff_hz  = fo["cutoff_hz"].as<double>(10.0);
    return cfg;
}

// ── Desired trajectory: sinusoidal tracking around home position ───────────────
// q_des[i](t) = q_home[i] + amp[i]*sin(2*pi*freq*t + phase[i])
static void computeDesired(const arm::VecN& q_home, double t, double freq_hz, double amp_rad, arm::VecN& q_des, arm::VecN& dq_des)
{
    int n = static_cast<int>(q_home.size());
    q_des  = q_home;
    dq_des = arm::VecN::Zero(n);
    for (int i = 0; i < n; ++i)
    {
        // Staggered phases so joints move independently.
        double phase = i * (M_PI / 3.0);
        q_des[i]  = q_home[i] + amp_rad * std::sin(2.0 * M_PI * freq_hz * t + phase);
        dq_des[i] = amp_rad * 2.0 * M_PI * freq_hz
                    * std::cos(2.0 * M_PI * freq_hz * t + phase);
    }
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    std::string cfg_path = "config/sim/piper_arm.yaml";
    if (argc > 1) cfg_path = argv[1];

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(cfg_path);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Cannot load " << cfg_path << ": " << e.what() << "\n";
        return 1;
    }

    // Resolve relative paths so the binary can be run from build/ or project root.
    // Strategy: try CWD first; if the file doesn't exist there, try relative to
    // the executable's parent directory (= project root when binary is in build/).
    namespace fs = std::filesystem;
    fs::path exe_root = fs::canonical("/proc/self/exe").parent_path().parent_path();
    auto resolve = [&](const std::string& p) -> std::string
    {
        fs::path fp(p);
        if (fp.is_absolute()) return p;
        if (fs::exists(fp)) return fp.lexically_normal().string();
        return (exe_root / fp).lexically_normal().string();
    };

    YAML::Node sim_node = root["simulation"];
    const std::string model_path = resolve(sim_node["model_path"].as<std::string>(
        "model/piper_torque_scene.xml"));
    const double duration  = sim_node["duration"].as<double>(20.0);
    const bool   visualize = sim_node["visualize"].as<bool>(true);
    const std::string log_path = resolve(sim_node["log_path"].as<std::string>(
        "data/logs/piper_arm.csv"));
    const bool realtime = sim_node["realtime"].as<bool>(true);
    const double realtime_factor = sim_node["realtime_factor"].as<double>(1.0);

    // Ensure log directory exists.
    fs::create_directories(fs::path(log_path).parent_path());

    // ── Simulation environment ───────────────────────────────────────────────
    sim::ArmEnv env(model_path);
    env.resetToKeyframe(0);

    // ── Build ArmConfig (resolves MuJoCo IDs) ────────────────────────────────
    arm::ArmConfig arm_cfg = loadArmConfig(root, env);
    int n = arm_cfg.n_joints;

    int ee_body_id = -1;
    try
    {
        ee_body_id = env.bodyId("link6");
    }
    catch (const std::exception& e)
    {
        std::cerr << "[warn] Could not resolve EE body 'link6': " << e.what() << "\n";
    }

    // ── Modules ──────────────────────────────────────────────────────────────
    arm::PDController    ctrl(loadPD(root, n));
    arm::JointSensor     sensor(loadSensor(root, n), n);
    arm::FrictionModel   friction(loadFriction(root, n));
    arm::DisturbanceManager disturbance(loadMatched(root, n),
                                        loadMismatched(root));
    disturbance.saveNominalState(env.model());

    arm::DOB                dob(loadDOB(root, n));
    arm::MomentumObserver   mom_obs(loadMO(root, n));
    arm::ForceObserver      force_obs(loadFO(root));
    dob.reset(n);
    mom_obs.reset(n);
    force_obs.reset(n);

    // ── Select which observer feeds the controller ────────────────────────────
    // "dob" | "momentum" | "force"  (config key: active_observer)
    const std::string active_obs = root["active_observer"].as<std::string>("dob");

    // ── Home position from keyframe ───────────────────────────────────────────
    arm::VecN q_home(n);
    for (int i = 0; i < n; ++i)
        q_home[i] = env.data()->qpos[arm_cfg.qpos_ids[i]];

    YAML::Node traj = root["trajectory"];
    double traj_amp  = traj["amplitude_rad"].as<double>(0.0);
    double traj_freq = traj["frequency_hz"].as<double>(0.1);

    const double dt = arm_cfg.dt;

    // ── Visualizer ───────────────────────────────────────────────────────────
    std::unique_ptr<sim::ArmVisualizer> vis;
    if (visualize)
    {
        try
        {
            vis = std::make_unique<sim::ArmVisualizer>(env);
        }
        catch (const std::exception& e)
        {
            std::cerr << "[warn] Visualizer disabled: " << e.what() << "\n";
        }
    }

    // ── Log file ──────────────────────────────────────────────────────────────
    std::ofstream log(log_path);
    if (!log) std::cerr << "[warn] Cannot open log: " << log_path << "\n";
    if (log)
    {
        log << "t";
        for (int i = 0; i < n; ++i) log << ",q" << i;
        for (int i = 0; i < n; ++i) log << ",dq" << i;
        for (int i = 0; i < n; ++i) log << ",q_des" << i;
        for (int i = 0; i < n; ++i) log << ",tau_ctrl" << i;
        for (int i = 0; i < n; ++i) log << ",d_hat" << i;
        log << ",ee_pos_x,ee_pos_y,ee_pos_z,ee_des_pos_x,ee_des_pos_y,ee_des_pos_z";
        log << "\n";
    }

    arm::VecN tau_ctrl   = arm::VecN::Zero(n);
    arm::VecN tau_applied = arm::VecN::Zero(n);

    std::cout << "Simulation started  duration=" << duration
              << " s  dt=" << dt << " s  n_joints=" << n << "\n"
              << "Active observer: " << active_obs << "\n";

    using Clock = std::chrono::steady_clock;
    const auto wall_start = Clock::now();

    // ── Main loop ─────────────────────────────────────────────────────────────
    while (env.time() < duration)
    {
        if (vis && vis->shouldClose()) break;

        double t = env.time();

        // 1. Ground-truth state from MuJoCo.
        arm::ArmState true_state = env.getState(arm_cfg.qpos_ids, arm_cfg.dof_ids);
        true_state.tau_meas      = env.getActuatorTorques(arm_cfg.dof_ids);

        // 2. Apply mismatched disturbance (external forces, mass/gravity changes).
        disturbance.applyMismatched(env.model(), env.data());

        // 3. Sensor layer — noisy measurement.
        arm::ArmState meas = sensor.read(true_state, dt);

        // 4. Mass matrix for observers (computed after last step).
        arm::MatN M_q = env.getMassMatrix();

        // 5. Update all observers.
        dob.update(meas, tau_ctrl, M_q, dt);
        mom_obs.update(meas, tau_ctrl, M_q, dt);
        force_obs.update(meas, tau_ctrl, M_q, dt);

        // 6. Select active observer output for the controller.
        arm::VecN d_hat = arm::VecN::Zero(n);
        if      (active_obs == "momentum") d_hat = mom_obs.estimate().head(n);
        else if (active_obs == "force")    d_hat = force_obs.estimate().head(n);
        else                               d_hat = dob.estimate().head(n);

        // 7. Desired trajectory.
        arm::VecN q_des(n), dq_des(n);
        computeDesired(q_home, t, traj_freq, traj_amp, q_des, dq_des);

        Eigen::Vector3d ee_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d ee_des_pos = Eigen::Vector3d::Zero();
        if (ee_body_id >= 0)
        {
            ee_pos = env.bodyPositionFromQ(true_state.q, arm_cfg.qpos_ids, ee_body_id);
            ee_des_pos = env.bodyPositionFromQ(q_des, arm_cfg.qpos_ids, ee_body_id);
        }

        // 8. Controller.
        tau_ctrl = ctrl.compute(meas, q_des, dq_des, d_hat);

        // 9. True friction (unknown to controller — represents real plant).
        arm::VecN tau_fric = friction.compute(true_state.dq);

        // 10. Matched disturbance (noise, constant bias, chirp) on top of friction.
        tau_applied = disturbance.applyMatched(tau_ctrl + tau_fric, t);

        // 11. Inject torques into MuJoCo motor actuators.
        env.applyTorques(tau_applied, arm_cfg.actuator_ids);

        // 12. Step simulation.
        env.step();

        // 13. Log.
        if (log)
        {
            log << t;
            for (int i = 0; i < n; ++i) log << "," << true_state.q[i];
            for (int i = 0; i < n; ++i) log << "," << true_state.dq[i];
            for (int i = 0; i < n; ++i) log << "," << q_des[i];
            for (int i = 0; i < n; ++i) log << "," << tau_ctrl[i];
            for (int i = 0; i < n; ++i) log << "," << d_hat[i];
            log << "," << ee_pos[0] << "," << ee_pos[1] << "," << ee_pos[2];
            log << "," << ee_des_pos[0] << "," << ee_des_pos[1] << "," << ee_des_pos[2];
            log << "\n";
        }

        // 14. Render.
        if (vis)
        {
            sim::ArmDisplay disp;
            disp.t           = t;
            disp.n_joints    = n;
            disp.true_state  = true_state;
            disp.meas_state  = meas;
            disp.q_des       = q_des;
            disp.tau_ctrl    = tau_ctrl;
            disp.tau_applied = tau_applied;
            disp.d_hat       = d_hat;
            disp.ee_pos      = ee_pos;
            disp.ee_des_pos  = ee_des_pos;
            vis->render(disp);
        }

        if (realtime && dt > 0.0)
        {
            const auto target = wall_start + std::chrono::duration<double>(env.time() / realtime_factor);
            std::this_thread::sleep_until(target);
        }
    }

    std::cout << "Simulation finished  t=" << env.time() << " s\n";
    if (log) std::cout << "Log: " << log_path << "\n";
    return 0;
}
