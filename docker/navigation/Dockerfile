# Navigation area: what the G1 needs to map, localize and move, the
# counterpart of home2's docker/navigation.
#   - lidar pipeline (body crop, /scan), FAST-LIO, slam_toolbox, Nav2,
#     nav_central / semantic_nav (g1_perception, g1_navigation)
#   - on the real robot also the MID-360 driver and g1_loco_bridge (Unitree's
#     LocoClient), so this one container drives the G1 like home2's navigation
#     container drives FRIDA's base (omnidriver)
#
# Built on home2's base image so that on the G1's Orin it shares its layers
# with home2's vision and hri images, which are to run there too.
ARG BASE_IMAGE=roborregos/home2:cpu_base
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=jazzy
USER ros
SHELL ["/bin/bash", "-c"]

RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-regulated-pure-pursuit-controller \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rviz2 \
    python3-pyqt5 python3-numpy python3-yaml \
    libeigen3-dev libpcl-dev cmake build-essential git \
    && sudo rm -rf /var/lib/apt/lists/*

# The rest of the dependencies, from the G1 packages' manifests. Gazebo is the
# simulation container's (g1_description declares it for the sim side).
COPY --chown=ros:ros src /tmp/deps/src
COPY --chown=ros:ros vendor/home2/frida_interfaces/package.xml /tmp/deps/frida_interfaces/package.xml
RUN sudo apt-get update && (sudo rosdep init || true) && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/deps --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
       --skip-keys "frida_interfaces frida_constants frida_gz_sim livox_ros_driver2 fast_lio unitree_sdk2 \
                    gz_ros2_control ros_gz_sim ros_gz_bridge g1_rl_bridge g1_sim_bringup" \
    ; sudo rm -rf /tmp/deps /var/lib/apt/lists/*

# unitree_sdk2 for g1_loco_bridge, at the commit third_party/ uses.
ARG UNITREE_SDK2_COMMIT=8e1b927586712edd6a4c29d50dd52dff1e163439
RUN git clone https://github.com/unitreerobotics/unitree_sdk2.git /tmp/unitree_sdk2 \
    && git -C /tmp/unitree_sdk2 checkout ${UNITREE_SDK2_COMMIT} \
    && cmake -S /tmp/unitree_sdk2 -B /tmp/unitree_sdk2/build -DBUILD_EXAMPLES=OFF \
    && sudo cmake --build /tmp/unitree_sdk2/build -j"$(nproc)" --target install \
    && sudo ldconfig && sudo rm -rf /tmp/unitree_sdk2

# Livox-SDK2, livox_ros_driver2 and FAST-LIO, with third_party/fetch_lio.sh
# (same pinned commits and patches) into /home/ros/lio_ws.
COPY --chown=ros:ros third_party/fetch_lio.sh /tmp/fetch_lio.sh
RUN LIO_WS=/home/ros/lio_ws bash /tmp/fetch_lio.sh --no-build \
    && sudo apt-get update \
    && rosdep install --from-paths /home/ros/lio_ws/src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
    ; sudo rm -rf /var/lib/apt/lists/*
RUN LIO_WS=/home/ros/lio_ws JOBS="$(nproc)" bash /tmp/fetch_lio.sh \
    && rm -rf /home/ros/lio_ws/build /home/ros/lio_ws/log /home/ros/lio_ws/deps /tmp/fetch_lio.sh
RUN echo "source /home/ros/lio_ws/install/setup.bash" >> /home/ros/.bashrc

# Network-only DDS (no iceoryx SHM, so no RouDi container needed); the
# simulation container, on plain ros:jazzy, can't do SHM either.
ENV CYCLONE_SHM=0
ENV G1_LIO_SETUP=/home/ros/lio_ws/install/setup.bash
ENV G1_INSTALL_DIR=/workspace/install
ENV G1_REPO=/workspace/src

WORKDIR /workspace
