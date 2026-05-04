#include "controller/arm_pd.hpp"
#include "robot_arm/joint_noise.hpp"
#include "mujoco_env.hpp"
#include "visualizer.hpp"

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

// ---------------------------------------------------------------------------
// Config helpers
// ---------------------------------------------------------------------------
static arm::ArmPdParams loadArmPd(const YAML::Node& doc, int njoints)
{
    arm::ArmPdParams p(njoints);
    try
    {
        const YAML::Node& pd = doc["arm_pd"];
        if (!pd) return p;
        p.tau_max = pd["tau_max"].as<double>(p.tau_max);
        if (pd["Kp"] && pd["Kp"].IsSequence())
            for (int i = 0; i < njoints && i < (int)pd["Kp"].size(); ++i)
                p.Kp[i] = pd["Kp"][i].as<double>(p.Kp[i]);
        if (pd["Kd"] && pd["Kd"].IsSequence())
            for (int i = 0; i < njoints && i < (int)pd["Kd"].size(); ++i)
                p.Kd[i] = pd["Kd"][i].as<double>(p.Kd[i]);
    }
    catch (...) { std::cerr << "[warn] arm_pd params not found, using defaults\n"; }
    return p;
}

static arm::JointNoiseParams loadJointNoise(const YAML::Node& doc)
{
    arm::JointNoiseParams p;
    try
    {
        const YAML::Node& jn = doc["joint_noise"];
        if (!jn) return p;
        p.pos_noise = jn["pos_noise"].as<double>(p.pos_noise);
        p.vel_noise = jn["vel_noise"].as<double>(p.vel_noise);
        p.tau_noise = jn["tau_noise"].as<double>(p.tau_noise);
    }
    catch (...) {}
    return p;
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv)
// ---------------------------------------------------------------------------
{
    std::string cfg_path = "config/sim/piper_arm.yaml";
    if (argc > 1) cfg_path = argv[1];

    YAML::Node doc, sim;
    try
    {
        doc = YAML::LoadFile(cfg_path);
        sim = doc["simulation"];
    }
    catch (const std::exception& e)
    {
        std::cerr << "Cannot load " << cfg_path << ": " << e.what() << "\n";
        return 1;
    }

    const std::string model_path = sim["model_path"].as<std::string>("model/piper_scene.xml");
    const std::string log_path   = sim["log_path"].as<std::string>("data/logs/arm_log.csv");
    const double      duration   = sim["duration"].as<double>(10.0);
    const bool        visualize  = sim["visualize"].as<bool>(true);
    const int         njoints    = sim["njoints"].as<int>(6);
    const std::string ee_body    = sim["ee_body"].as<std::string>("link6");
    const std::string trk_body   = sim["track_body"].as<std::string>("link_base");

    // ── MuJoCo environment ─────────────────────────────────────────────────
    sim::MujocoEnv env(model_path);
    env.reset();

    // ── Controller + noise ─────────────────────────────────────────────────
    arm::ArmPdController ctrl(loadArmPd(doc, njoints));
    arm::JointNoise      noise(loadJointNoise(doc));
    const bool noise_enabled = doc["joint_noise"]["enabled"].as<bool>(false);

    // ── Target joint pose ──────────────────────────────────────────────────
    Eigen::VectorXd q_des  = Eigen::VectorXd::Zero(njoints);
    Eigen::VectorXd dq_des = Eigen::VectorXd::Zero(njoints);
    try
    {
        const YAML::Node& tgt = doc["target_joints"];
        if (tgt && tgt.IsSequence())
            for (int i = 0; i < njoints && i < (int)tgt.size(); ++i)
                q_des[i] = tgt[i].as<double>(0.0);
    }
    catch (...) {}

    // ── Visualizer ─────────────────────────────────────────────────────────
    std::unique_ptr<sim::Visualizer> vis;
    if (visualize)
    {
        try { vis = std::make_unique<sim::Visualizer>(env, trk_body); }
        catch (const std::exception& e)
        {
            std::cerr << "[warn] Visualizer disabled: " << e.what() << "\n";
        }
    }

    // ── Log file ───────────────────────────────────────────────────────────
    std::ofstream log(log_path);
    if (!log) std::cerr << "[warn] Cannot open log: " << log_path << "\n";
    else
    {
        log << "t";
        for (int i = 0; i < njoints; ++i) log << ",q" << (i + 1);
        for (int i = 0; i < njoints; ++i) log << ",dq" << (i + 1);
        for (int i = 0; i < njoints; ++i) log << ",tau" << (i + 1);
        log << ",ee_x,ee_y,ee_z\n";
    }

    std::cout << "ARM simulation started  model=" << model_path
              << "  duration=" << duration
              << " s  dt=" << env.model()->opt.timestep << " s\n"
              << "njoints=" << njoints << "  ee=" << ee_body
              << "  noise=" << (noise_enabled ? "on" : "off") << "\n";

    // ── Main control loop ──────────────────────────────────────────────────
    while (env.time() < duration)
    {
        if (vis && vis->shouldClose()) break;

        // 1. Sense (ground truth from MuJoCo)
        arm::ArmState raw(njoints);
        raw.q  = env.getJointPos(njoints);
        raw.dq = env.getJointVel(njoints);

        // 2. Inject sensor noise
        const arm::ArmState& sensed = noise_enabled
                                          ? noise.addSensorNoise(raw)
                                          : raw;

        // 3. PD control:  tau = Kp*(q_des - q) + Kd*(dq_des - dq)
        Eigen::VectorXd tau = ctrl.compute(sensed, q_des, dq_des);

        // 4. Inject torque disturbance
        if (noise_enabled) tau = noise.disturbTorque(tau);

        // 5. Apply torques and advance simulation
        env.setControls(tau);
        env.step();

        // 6. Log ground-truth state
        const double t = env.time();
        if (log)
        {
            drone::Vec3 ee = env.getBodyPos(ee_body);
            log << t;
            for (int i = 0; i < njoints; ++i) log << ',' << raw.q[i];
            for (int i = 0; i < njoints; ++i) log << ',' << raw.dq[i];
            for (int i = 0; i < njoints; ++i) log << ',' << tau[i];
            log << ',' << ee.x() << ',' << ee.y() << ',' << ee.z() << '\n';
        }

        // 7. Render
        if (vis)
        {
            sim::ArmDisplay disp;
            disp.time    = t;
            disp.state   = sensed;
            disp.torques = tau;
            disp.q_des   = q_des;
            disp.ee_pos  = env.getBodyPos(ee_body);
            vis->render(disp);
        }
    }

    std::cout << "ARM simulation finished  t=" << env.time() << " s\n";
    if (log) std::cout << "Log saved to " << log_path << "\n";
    return 0;
}
