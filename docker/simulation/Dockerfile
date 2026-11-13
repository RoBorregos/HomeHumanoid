# Simulation area: the G1 in Gazebo (kinematic base + simulated MID-360) and
# in MuJoCo (Unitree's RL walking policy), the counterpart of home2's
# docker/simulation. Navigation runs in its own container on top of either,
# exactly as it does on the real robot.
#
# On plain ros:jazzy like home2's simulation image: Gazebo Harmonic comes from
# the ROS archive and nothing else of home2's base is needed. Development
# machines only; never runs on the robot.
ARG BASE_IMAGE=ros:jazzy
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=jazzy
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8

RUN (id -u ubuntu >/dev/null 2>&1 && userdel -r ubuntu || true) \
    && (getent group ubuntu >/dev/null 2>&1 && groupdel ubuntu || true) \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic + the bridges, and what unitree_mujoco / g1_ctrl link
# (third_party/README.md). xdotool + Xvfb: release_band.py drives the MuJoCo
# window, also with no display.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rviz2 \
    ros-dev-tools \
    python3-numpy python3-yaml \
    libeigen3-dev libyaml-cpp-dev libspdlog-dev libfmt-dev libboost-program-options-dev \
    libglfw3-dev \
    cmake build-essential git curl ca-certificates \
    xdotool xvfb x11-utils mesa-utils libegl1 libgl1 \
    && rm -rf /var/lib/apt/lists/*

COPY src /tmp/deps/src
COPY vendor/home2/frida_interfaces/package.xml /tmp/deps/frida_interfaces/package.xml
RUN apt-get update && (rosdep init || true) && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/deps --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
       --skip-keys "frida_interfaces frida_constants frida_gz_sim livox_ros_driver2 fast_lio unitree_sdk2" \
    ; rm -rf /tmp/deps /var/lib/apt/lists/*

# unitree_sdk2, installed twice because unitree_mujoco and unitree_rl_lab look
# for it in different prefixes (third_party/README.md).
ARG UNITREE_SDK2_COMMIT=8e1b927586712edd6a4c29d50dd52dff1e163439
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git /tmp/unitree_sdk2 \
    && git -C /tmp/unitree_sdk2 checkout ${UNITREE_SDK2_COMMIT} \
    && cmake -S /tmp/unitree_sdk2 -B /tmp/unitree_sdk2/build_opt -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics -DBUILD_EXAMPLES=OFF \
    && cmake --build /tmp/unitree_sdk2/build_opt -j"$(nproc)" --target install \
    && cmake -S /tmp/unitree_sdk2 -B /tmp/unitree_sdk2/build_local -DBUILD_EXAMPLES=OFF \
    && cmake --build /tmp/unitree_sdk2/build_local -j"$(nproc)" --target install \
    && ldconfig && rm -rf /tmp/unitree_sdk2

# Pinned downloads with retries (docker/scripts/fetch.sh): one layer each, so a
# dropped connection doesn't redo the ones already done.
COPY docker/scripts/fetch.sh /usr/local/bin/fetch.sh

# MuJoCo + unitree_mujoco, at the versions used on the Orin.
ARG MUJOCO_VERSION=3.17.0
ARG UNITREE_MUJOCO_COMMIT=1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d
RUN fetch.sh tar "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-$(uname -m).tar.gz" /opt
RUN fetch.sh git https://github.com/unitreerobotics/unitree_mujoco.git ${UNITREE_MUJOCO_COMMIT} /opt/unitree_mujoco
RUN ln -s /opt/mujoco-${MUJOCO_VERSION} /opt/unitree_mujoco/simulate/mujoco \
    && cmake -S /opt/unitree_mujoco/simulate -B /opt/unitree_mujoco/simulate/build \
    && cmake --build /opt/unitree_mujoco/simulate/build -j"$(nproc)" --target unitree_mujoco
# Only the simulator: its jstest joystick tool misses <cstdint> and does not
# build with GCC 13 (Ubuntu 24.04), and nothing here uses it.
COPY docker/simulation/unitree_mujoco_config.yaml /opt/unitree_mujoco/simulate/config.yaml

# unitree_rl_lab's deploy controller (g1_ctrl) only; no Isaac Lab. Its
# vendored onnxruntime is x86_64: on aarch64 swap in the matching release.
ARG UNITREE_RL_LAB_COMMIT=4960b84732b0c2ec593dccbfe963fda1bcd7b1e3
ARG ONNXRUNTIME_VERSION=1.22.0
RUN fetch.sh git https://github.com/unitreerobotics/unitree_rl_lab.git ${UNITREE_RL_LAB_COMMIT} /opt/unitree_rl_lab
RUN if [ "$(uname -m)" = aarch64 ]; then \
         rm -rf /opt/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION} \
         && fetch.sh tar "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}.tgz" \
              /opt/unitree_rl_lab/deploy/thirdparty \
         && sed -i "s/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}/onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}/g" \
              /opt/unitree_rl_lab/deploy/robots/g1_29dof/CMakeLists.txt; \
       fi \
    && cmake -S /opt/unitree_rl_lab/deploy/robots/g1_29dof -B /opt/unitree_rl_lab/deploy/robots/g1_29dof/build \
    && cmake --build /opt/unitree_rl_lab/deploy/robots/g1_29dof/build -j"$(nproc)"
# start_mujoco.sh widens deploy.yaml's command ranges at run time: the ros user
# needs to write it.
RUN chown -R ${USERNAME}:${USERNAME} /opt/unitree_rl_lab/deploy/robots/g1_29dof/config /opt/unitree_mujoco/simulate

COPY vendor/home2/docker/cyclonedds_setup.sh /usr/local/bin/cyclonedds_setup.sh
RUN chmod +x /usr/local/bin/cyclonedds_setup.sh && /usr/local/bin/cyclonedds_setup.sh
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

ENV UNITREE_MUJOCO_DIR=/opt/unitree_mujoco
ENV UNITREE_RL_LAB_DIR=/opt/unitree_rl_lab
ENV G1_INSTALL_DIR=/workspace/install
ENV G1_REPO=/workspace/src

RUN mkdir -p /workspace && chown ${USERNAME}:${USERNAME} /workspace
USER ${USERNAME}
WORKDIR /workspace
