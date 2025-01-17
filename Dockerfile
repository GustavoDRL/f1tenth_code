FROM ros:humble
SHELL ["/bin/bash", "-c"]

# Install system dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y \
    python3-pip \
    python3-dev \
    nano \
    python3-numpy \
    python3-sklearn \
    python3-matplotlib \
    cmake \
    g++ \
    git \
    curl \
    wget \
    libeigen3-dev \
    libboost-all-dev \
    ca-certificates \
    python3-rosdep \
    python-is-python3 && \
    rm -rf /var/lib/apt/lists/*

# Install ROS2 dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    ros-humble-nav2-* \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    sudo apt install ros-humble-turtlebot3\
    ros-humble-cartographer\
    ros-humble-cartographer-ros\
    ros-humble-ackermann-msgs \
    ros-humble-diagnostic-msgs \
    ros-humble-lifecycle-msgs \
    ros-humble-xacro\
    ros-humble-teleop-twist-keyboard && \
    rm -rf /var/lib/apt/lists/*

# Install Python packages and upgrade
RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get install -y \
    python3-ament-lint \
    python3-colcon-common-extensions && \
    pip3 install --no-cache-dir \
    transforms3d \
    numpy

# Initialize rosdep
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update

# Clone and install F1TENTH Gym
RUN git config --global http.sslVerify false && \
    git clone https://github.com/f1tenth/f1tenth_gym && \
    cd f1tenth_gym && \
    pip3 install -e .

# Create workspace and copy source files
RUN mkdir -p sim_ws/src
COPY src/ /sim_ws/src/

# Build the workspace
RUN . /opt/ros/humble/setup.bash && \
    cd sim_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro humble -y || true 
    

# Set up environment
RUN echo "export TERM=xterm-256color" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /sim_ws/install/setup.bash" >> ~/.bashrc

ARG TARGETPLATFORM

WORKDIR '/sim_ws'
ENTRYPOINT ["/bin/bash"]