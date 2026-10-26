FROM ros:foxy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy
ENV ROS_DOMAIN_ID=10
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_LOCALHOST_ONLY=0

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# nstall system and ROS dependencies
# Do NOT redirect Ubuntu repositories to old-releases
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    curl \
    ca-certificates \
    pkg-config \
    libyaml-cpp-dev \
    libasio-dev \
    libtinyxml2-dev \
    liburdfdom-dev \
    libconsole-bridge-dev \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    ros-foxy-ros2-control \
    ros-foxy-ros2-controllers \
    ros-foxy-ros2controlcli \
    ros-foxy-xacro \
    ros-foxy-rviz2 \
    ros-foxy-tf2-tools \
    ros-foxy-moveit \
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-robot-state-publisher \
    ros-foxy-joint-state-publisher \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-control-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init 2>/dev/null || true
RUN rosdep update

# Create workspace and clone the Foxy driver
WORKDIR /root/ros2_ws
RUN mkdir -p src

ARG UR_DRIVER_BRANCH=foxy

RUN git clone \
    --single-branch \
    --branch "${UR_DRIVER_BRANCH}" \
    https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git \
    src/Universal_Robots_ROS2_Driver

# Import repositories required by the driver

RUN if [ -f src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.repos ]; then \
      vcs import src \
        --skip-existing \
        --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.repos; \
    else \
      echo "No driver .repos file found; continuing with cloned sources."; \
    fi

# Install dependencies
RUN source /opt/ros/foxy/setup.bash && \
    rosdep install \
      --from-paths src \
      --ignore-src \
      --rosdistro foxy \
      --skip-keys="libyaml-cpp-dev" \
      -r -y

#  Build the driver and its required dependencies

RUN source /opt/ros/foxy/setup.bash && \
    colcon build \
      --packages-up-to ur_robot_driver ur_moveit_config \
      --merge-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release

# Check package version and configure shell

RUN source /opt/ros/foxy/setup.bash && \
    source /root/ros2_ws/install/setup.bash && \
    ros2 pkg prefix ur_robot_driver

RUN printf '\nsource /opt/ros/foxy/setup.bash\n' >> /root/.bashrc && \
    printf 'if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi\n' >> /root/.bashrc

WORKDIR /root/ros2_ws

CMD ["bash", "-lc", "source /opt/ros/foxy/setup.bash && source /root/ros2_ws/install/setup.bash && sleep infinity"]
