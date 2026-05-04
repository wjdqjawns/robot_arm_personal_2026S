cmake_minimum_required(VERSION 3.18)
project(prj_arm CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MuJoCo is built from source at ~/mujoco/install.
# Override with -DMUJOCO_ROOT=<path> if installed elsewhere.
set(MUJOCO_ROOT "$ENV{HOME}/mujoco/install" CACHE PATH "MuJoCo install prefix")
list(APPEND CMAKE_PREFIX_PATH "${MUJOCO_ROOT}")

find_package(mujoco   REQUIRED)
find_package(Eigen3   REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glfw3    REQUIRED)
find_package(OpenGL   REQUIRED)

# Keep the binary runnable without LD_LIBRARY_PATH.
set(CMAKE_INSTALL_RPATH "${MUJOCO_ROOT}/lib")
set(CMAKE_BUILD_RPATH   "${MUJOCO_ROOT}/lib")

# ── Shared sim layer ──────────────────────────────────────────────────────────
set(SIM_SRCS
  sim/mujoco_env.cpp
  sim/visualizer.cpp
)

# ── Research-arm module sources ───────────────────────────────────────────────
set(ARM_MODULE_SRCS
  src/disturbance.cpp
  src/sensor.cpp
  src/friction.cpp
  src/observer/dob.cpp
  src/observer/momentum_observer.cpp
  src/observer/force_observer.cpp
  src/controller/pd_controller.cpp
)

# ── arm_sim binary ─────────────────────────────────────────────────────────────
add_executable(arm_sim
  src/main.cpp
  ${ARM_MODULE_SRCS}
  ${SIM_SRCS}
)

target_include_directories(arm_sim PRIVATE include sim)

target_link_libraries(arm_sim
  mujoco::mujoco
  Eigen3::Eigen
  yaml-cpp
  glfw
  OpenGL::GL
)

# Bake plugin directory path so mujoco_env.cpp can load mesh decoders.
target_compile_definitions(arm_sim PRIVATE
  MUJOCO_PLUGIN_DIR="${MUJOCO_ROOT}/bin/mujoco_plugin"
)
