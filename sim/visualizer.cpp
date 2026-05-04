#include "visualizer.hpp"
#include <stdexcept>
#include <cstdio>
#include <cstring>
#include <algorithm>

namespace sim
{
    // ── Static singleton ──────────────────────────────────────────────────────────
    ArmVisualizer* ArmVisualizer::instance_ = nullptr;

    // ── PlotHistory ───────────────────────────────────────────────────────────────
    void ArmPlotHistory::push(const ArmDisplay& d)
    {
        // Track joint-0 for the real-time plot.
        double qe = (d.q_des.size() > 0 && d.true_state.q.size() > 0) ? d.q_des[0] - d.true_state.q[0] : 0.0;

        t_vals.push_back(static_cast<float>(d.t));
        q_true.push_back(static_cast<float>(d.true_state.q.size() > 0 ? d.true_state.q[0] : 0.0));
        q_des.push_back(static_cast<float>(d.q_des.size() > 0 ? d.q_des[0] : 0.0));
        q_err.push_back(static_cast<float>(qe));
        tau_ctrl.push_back(static_cast<float>(d.tau_ctrl.size() > 0 ? d.tau_ctrl[0] : 0.0));
        d_hat.push_back(static_cast<float>(d.d_hat.size() > 0 ? d.d_hat[0] : 0.0));

        while (static_cast<int>(t_vals.size()) > MAX)
        {
            t_vals.pop_front();
            q_true.pop_front(); q_des.pop_front(); q_err.pop_front();
            tau_ctrl.pop_front(); d_hat.pop_front();
        }
    }

    void ArmTrajectoryHistory::push(const Eigen::Vector3d& actual_pos, const Eigen::Vector3d& desired_pos)
    {
        actual.push_back({static_cast<float>(actual_pos.x()), static_cast<float>(actual_pos.y()), static_cast<float>(actual_pos.z())});
        desired.push_back({static_cast<float>(desired_pos.x()), static_cast<float>(desired_pos.y()), static_cast<float>(desired_pos.z())});

        while (static_cast<int>(actual.size()) > MAX)
        {
            actual.pop_front();
            desired.pop_front();
        }
    }

    // ── Constructor / destructor ──────────────────────────────────────────────────
    ArmVisualizer::ArmVisualizer(ArmEnv& env, int width, int height) : env_(env)
    {
        instance_ = this;

        if (!glfwInit())
            throw std::runtime_error("glfwInit failed");

        glfwWindowHint(GLFW_SAMPLES, 4);
        window_ = glfwCreateWindow(width, height, "Piper Arm Research Sim", nullptr, nullptr);
        if (!window_)
        {
            glfwTerminate(); throw std::runtime_error("glfwCreateWindow failed");
        }

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        glfwSetKeyCallback(window_,         keyCb);
        glfwSetScrollCallback(window_,      scrollCb);
        glfwSetMouseButtonCallback(window_, mouseBtnCb);
        glfwSetCursorPosCallback(window_,   cursorCb);

        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);
        mjv_defaultFigure(&fig_);

        mjv_makeScene(env_.model(), &scn_, 5000);
        mjr_makeContext(env_.model(), &con_, mjFONTSCALE_150);

        // Set up the figure for joint-0 tracking plot.
        fig_.flg_legend        = 1;
        fig_.flg_ticklabel[0]  = 1;
        fig_.flg_ticklabel[1]  = 1;
        fig_.linewidth         = 2.0f;
        std::snprintf(fig_.title,   sizeof(fig_.title),   "Joint 0");
        std::snprintf(fig_.xlabel,  sizeof(fig_.xlabel),  "time (s)");
        std::snprintf(fig_.xformat, sizeof(fig_.xformat), "%s", "%.1f");
        std::snprintf(fig_.yformat, sizeof(fig_.yformat), "%s", "%.3f");
        std::snprintf(fig_.linename[0], sizeof(fig_.linename[0]), "q");
        std::snprintf(fig_.linename[1], sizeof(fig_.linename[1]), "q_des");
        std::snprintf(fig_.linename[2], sizeof(fig_.linename[2]), "d_hat");
        fig_.linergb[0][0] = 0.35f; fig_.linergb[0][1] = 0.70f; fig_.linergb[0][2] = 0.95f;
        fig_.linergb[1][0] = 0.95f; fig_.linergb[1][1] = 0.75f; fig_.linergb[1][2] = 0.30f;
        fig_.linergb[2][0] = 0.90f; fig_.linergb[2][1] = 0.35f; fig_.linergb[2][2] = 0.35f;

        // Track link6 (EE) by default.
        int body_id = mj_name2id(env_.model(), mjOBJ_BODY, "link6");
        
        if (body_id >= 0)
        {
            cam_.type       = mjCAMERA_TRACKING;
            cam_.trackbodyid = body_id;
            cam_.distance   = 1.0;
            cam_.azimuth    = 135.0;
            cam_.elevation  = -20.0;
        }

        try
        {
            ee_body_id_ = env_.bodyId("link6");
        }
        catch (...)
        {
            ee_body_id_ = body_id;
        }
    }

    ArmVisualizer::~ArmVisualizer()
    {
        mjr_freeContext(&con_);
        mjv_freeScene(&scn_);
        if (window_) glfwDestroyWindow(window_);
        glfwTerminate();
        instance_ = nullptr;
    }

    // ── Public API ────────────────────────────────────────────────────────────────
    bool ArmVisualizer::shouldClose() const
    {
        return glfwWindowShouldClose(window_);
    }

    void ArmVisualizer::render(const ArmDisplay& d)
    {
        if (!window_) return;
        glfwPollEvents();

        history_.push(d);
        ee_traj_.push(d.ee_pos, d.ee_des_pos);

        int W, H;
        glfwGetFramebufferSize(window_, &W, &H);

        mjrRect viewport{0, 0, W, H};

        mjv_updateScene(env_.model(), env_.data(), &opt_, nullptr, &cam_,
                        mjCAT_ALL, &scn_);

        if (ee_body_id_ >= 0)
        {
            Eigen::Vector3d origin = env_.bodyPosition(ee_body_id_);
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    R(r, c) = env_.data()->xmat[9 * ee_body_id_ + 3 * r + c];

            const float actual_rgba[4]  = {0.90f, 0.20f, 0.20f, 1.0f};
            const float desired_rgba[4] = {0.15f, 0.80f, 0.25f, 1.0f};

            for (std::size_t i = 1; i < ee_traj_.actual.size(); ++i)
            {
                appendTrajectoryGeom(ee_traj_.actual[i - 1], ee_traj_.actual[i], actual_rgba);
            }
            for (std::size_t i = 1; i < ee_traj_.desired.size(); ++i)
            {
                appendTrajectoryGeom(ee_traj_.desired[i - 1], ee_traj_.desired[i], desired_rgba);
            }
            appendFrameGeom(origin, R);
        }

        mjr_render(viewport, &scn_, &con_);

        // Figure in bottom-right quarter.
        int fw = W / 3, fh = H / 3;
        mjrRect fig_rect{W - fw - 10, 10, fw, fh};
        updateFigure(d);
        mjr_figure(fig_rect, &fig_, &con_);

        renderOverlay(d, W, H);

        glfwSwapBuffers(window_);
    }

    // ── Private helpers ───────────────────────────────────────────────────────────
    void ArmVisualizer::updateFigure(const ArmDisplay& d)
    {
        const int n = static_cast<int>(history_.t_vals.size());
        if (n == 0) return;

        for (int k = 0; k < n; ++k)
        {
            fig_.linedata[0][2 * k]     = history_.t_vals[k];
            fig_.linedata[0][2 * k + 1] = history_.q_true[k];
            fig_.linedata[1][2 * k]     = history_.t_vals[k];
            fig_.linedata[1][2 * k + 1] = history_.q_des[k];
            fig_.linedata[2][2 * k]     = history_.t_vals[k];
            fig_.linedata[2][2 * k + 1] = history_.d_hat[k];
        }

        fig_.linepnt[0] = n;
        fig_.linepnt[1] = n;
        fig_.linepnt[2] = n;
    }

    void ArmVisualizer::appendTrajectoryGeom(const std::array<float, 3>& from, const std::array<float, 3>& to, const float rgba[4])
    {
        if (scn_.ngeom >= scn_.maxgeom) return;

        mjvGeom& geom = scn_.geoms[scn_.ngeom++];
        mjv_initGeom(&geom, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba);
        geom.category = mjCAT_DECOR;

        mjtNum p1[3] = {from[0], from[1], from[2]};
        mjtNum p2[3] = {to[0], to[1], to[2]};
        mjv_connector(&geom, mjGEOM_LINE, 2.0, p1, p2);
    }

    void ArmVisualizer::appendFrameGeom(const Eigen::Vector3d& origin, const Eigen::Matrix3d& R)
    {
        const float x_rgba[4] = {0.95f, 0.25f, 0.25f, 1.0f};
        const float y_rgba[4] = {0.20f, 0.85f, 0.30f, 1.0f};
        const float z_rgba[4] = {0.30f, 0.45f, 0.95f, 1.0f};
        const double frame_scale = 0.08;

        Eigen::Vector3d x_tip = origin + frame_scale * R.col(0);
        Eigen::Vector3d y_tip = origin + frame_scale * R.col(1);
        Eigen::Vector3d z_tip = origin + frame_scale * R.col(2);

        appendTrajectoryGeom(std::array<float, 3>{static_cast<float>(origin.x()), static_cast<float>(origin.y()), static_cast<float>(origin.z())},
                            std::array<float, 3>{static_cast<float>(x_tip.x()), static_cast<float>(x_tip.y()), static_cast<float>(x_tip.z())}, x_rgba);
        appendTrajectoryGeom(std::array<float, 3>{static_cast<float>(origin.x()), static_cast<float>(origin.y()), static_cast<float>(origin.z())},
                            std::array<float, 3>{static_cast<float>(y_tip.x()), static_cast<float>(y_tip.y()), static_cast<float>(y_tip.z())}, y_rgba);
        appendTrajectoryGeom(std::array<float, 3>{static_cast<float>(origin.x()), static_cast<float>(origin.y()), static_cast<float>(origin.z())},
                            std::array<float, 3>{static_cast<float>(z_tip.x()), static_cast<float>(z_tip.y()), static_cast<float>(z_tip.z())}, z_rgba);
    }

    void ArmVisualizer::renderOverlay(const ArmDisplay& d, int W, int H)
    {
        char buf[2048];
        int  pos = 0;
        int  n   = d.n_joints;

        pos += std::snprintf(buf + pos, sizeof(buf) - pos,
                            "t = %.3f s\n\n", d.t);
        pos += std::snprintf(buf + pos, sizeof(buf) - pos,
                            "%-6s  %8s  %8s  %8s  %8s  %8s\n",
                            "joint", "q[rad]", "q_des", "error", "tau_c", "d_hat");

        for (int i = 0; i < n; ++i)
        {
            double q = (d.true_state.q.size() > i)  ? d.true_state.q[i] : 0.0;
            double qd = (d.q_des.size() > i) ? d.q_des[i] : 0.0;
            double tc = (d.tau_ctrl.size() > i) ? d.tau_ctrl[i] : 0.0;
            double dh = (d.d_hat.size() > i) ? d.d_hat[i] : 0.0;
            pos += std::snprintf(buf + pos, sizeof(buf) - pos, "%-6d  %8.4f  %8.4f  %8.4f  %8.3f  %8.3f\n", i + 1, q, qd, qd - q, tc, dh);
        }

        mjrRect overlay{0, 0, W, H};
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, overlay, buf, nullptr, &con_);
    }

    // ── GLFW callbacks ────────────────────────────────────────────────────────────
    void ArmVisualizer::keyCb(GLFWwindow* w, int key, int, int action, int)
    {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
            glfwSetWindowShouldClose(w, GLFW_TRUE);
    }

    void ArmVisualizer::scrollCb(GLFWwindow*, double, double yoffset)
    {
        if (!instance_) return;
        instance_->cam_.distance = std::max(0.1, instance_->cam_.distance - yoffset * 0.05);
    }

    void ArmVisualizer::mouseBtnCb(GLFWwindow*, int button, int action, int)
    {
        if (!instance_) return;
        if (button == GLFW_MOUSE_BUTTON_LEFT)
            instance_->btn_left_  = (action == GLFW_PRESS);
        if (button == GLFW_MOUSE_BUTTON_RIGHT)
            instance_->btn_right_ = (action == GLFW_PRESS);
    }

    void ArmVisualizer::cursorCb(GLFWwindow* w, double xpos, double ypos)
    {
        if (!instance_) return;
        double dx = xpos - instance_->last_x_;
        double dy = ypos - instance_->last_y_;
        instance_->last_x_ = xpos;
        instance_->last_y_ = ypos;

        int W, H;
        glfwGetWindowSize(w, &W, &H);

        if (instance_->btn_left_)
        {
            instance_->cam_.azimuth  -= 0.3 * dx;
            instance_->cam_.elevation = std::clamp( instance_->cam_.elevation + 0.3 * dy, -90.0, 90.0);
        }
        if (instance_->btn_right_)
        {
            instance_->cam_.distance = std::max( 0.01, instance_->cam_.distance * (1.0 + dy * 0.01));
        }
    }
} // namespace sim
