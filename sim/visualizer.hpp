#pragma once
#include "mujoco_env.hpp"
#include "arm/types.hpp"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <array>
#include <deque>
#include <string>

namespace sim
{
    // Per-frame snapshot passed from the main loop to the visualizer.
    struct ArmDisplay
    {
        double         t{0};
        int            n_joints{6};
        arm::ArmState  true_state;    // ground truth from MuJoCo
        arm::ArmState  meas_state;    // noisy sensor measurement
        arm::VecN      q_des;         // desired joint positions
        arm::VecN      tau_ctrl;      // controller output (before matched disturbance)
        arm::VecN      tau_applied;   // actual torque injected into MuJoCo
        arm::VecN      d_hat;         // disturbance estimate from active observer
        Eigen::Vector3d ee_pos{Eigen::Vector3d::Zero()};
        Eigen::Vector3d ee_des_pos{Eigen::Vector3d::Zero()};
    };

    // Thin history ring for the real-time plot (joint 0 only for clarity).
    struct ArmPlotHistory
    {
        static constexpr int MAX = 800;

        std::deque<float> t_vals;
        std::deque<float> q_true;
        std::deque<float> q_des;
        std::deque<float> q_err;
        std::deque<float> tau_ctrl;
        std::deque<float> d_hat;

        void push(const ArmDisplay& d);
    };

    struct ArmTrajectoryHistory
    {
        static constexpr int MAX = 600;

        std::deque<std::array<float, 3>> actual;
        std::deque<std::array<float, 3>> desired;

        void push(const Eigen::Vector3d& actual_pos, const Eigen::Vector3d& desired_pos);
    };

    class ArmVisualizer
    {
    public:
        explicit ArmVisualizer(ArmEnv& env, int width = 1280, int height = 720);
        ~ArmVisualizer();

        ArmVisualizer(const ArmVisualizer&)            = delete;
        ArmVisualizer& operator=(const ArmVisualizer&) = delete;

        bool shouldClose() const;
        void render(const ArmDisplay& d);

    private:
        ArmEnv&     env_;
        GLFWwindow* window_{nullptr};
        mjvCamera   cam_{};
        mjvOption   opt_{};
        mjvScene    scn_{};
        mjrContext  con_{};
        mjvFigure   fig_{};

        ArmPlotHistory history_;
        ArmTrajectoryHistory ee_traj_;
        int ee_body_id_{-1};

        bool   btn_left_{false}, btn_right_{false};
        double last_x_{0}, last_y_{0};

        static ArmVisualizer* instance_;

        static void keyCb(GLFWwindow*, int key, int, int action, int);
        static void scrollCb(GLFWwindow*, double, double yoffset);
        static void mouseBtnCb(GLFWwindow*, int button, int action, int);
        static void cursorCb(GLFWwindow*, double xpos, double ypos);

        void renderOverlay(const ArmDisplay& d, int W, int H);
        void updateFigure(const ArmDisplay& d);
        void appendTrajectoryGeom(const std::array<float, 3>& from,
                                const std::array<float, 3>& to,
                                const float rgba[4]);
        void appendFrameGeom(const Eigen::Vector3d& origin,
                            const Eigen::Matrix3d& R);
    };
} // namespace sim
