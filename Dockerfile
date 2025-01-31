FROM ros:humble
SHELL ["/bin/bash", "-c"]

# Atualizar pacotes e corrigir dependências quebradas antes da instalação
RUN apt-get clean && apt-get update --fix-missing && apt-get upgrade -y && apt-get dist-upgrade -y

# Instalar dependências do sistema
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
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
    libsdl2-2.0-0 \
    libsdl2-dev \
    python-is-python3 && \
    rm -rf /var/lib/apt/lists/*

# Instalar pacotes de joystick separadamente
RUN apt-get update --fix-missing && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    joystick \
    jstest-gtk && \
    rm -rf /var/lib/apt/lists/*

# Instalar protobuf-compiler separadamente para evitar conflitos
RUN apt-get update && \
    apt-get install -y protobuf-compiler && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Instalar dependências do ROS2
RUN apt-get update --fix-missing && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-colcon-common-extensions \
    libeigen3-dev \
    libsdl2-dev \
    ros-humble-rviz2 \
    ros-humble-nav2-core \
    ros-humble-nav2-bringup \
    ros-humble-tf2-ros \
    ros-humble-ackermann-msgs \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Instalar pacotes Python necessários
RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get install -y \
    python3-ament-lint \
    python3-colcon-common-extensions && \
    pip3 install --no-cache-dir \
    transforms3d \
    numpy

# Inicializar rosdep
RUN rosdep init || echo "rosdep já foi inicializado" && \
    rosdep update

# Clonar e instalar o F1TENTH Gym
RUN git config --global http.sslVerify false && \
    git clone https://github.com/f1tenth/f1tenth_gym && \
    cd f1tenth_gym && \
    pip3 install -e .

# Criar workspace e copiar arquivos de código-fonte
RUN mkdir -p sim_ws/src
COPY src/ /sim_ws/src/

# Construir o workspace do ROS
RUN . /opt/ros/humble/setup.bash && \
    cd sim_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro humble -y || true 

# Configurar ambiente do ROS no bashrc
RUN echo "export TERM=xterm-256color" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /sim_ws/install/setup.bash" >> ~/.bashrc

# Install required packages for controller support
RUN apt-get update && apt-get install -y \
    joystick \
    ros-humble-joy \
    ros-humble-joy-linux \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Add udev rules for 8BitDo controller
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="2dc8", ATTRS{idProduct}=="310a", MODE="0666"' > /etc/udev/rules.d/99-8bitdo.rules


ARG TARGETPLATFORM

WORKDIR "/sim_ws"
ENTRYPOINT ["/bin/bash"]
