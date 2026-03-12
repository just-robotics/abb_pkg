FROM ros:humble-perception-jammy

ENV TZ=Europe/Moscow
ENV DEBIAN_FRONTEND=noninteractive

ENV SOURCE=/opt/ros/humble/setup.sh

ENV WORKDIR=/workspace
WORKDIR ${WORKDIR}

RUN apt update -y && \
    apt install -y \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-ament-package \
    libprotobuf-dev \
    protobuf-compiler \
    ros-humble-teleop-twist-joy \
    ros-humble-xacro \
    ros-humble-rviz2 \
    git \
    cmake \
    build-essential \
    ros-humble-rqt* \
    iputils-ping \
    iproute2 \
    vim \
    libnlopt-cxx-dev \
    libnlopt-dev \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install setuptools==58.2.0 pypylon

COPY ../workspace /workspace

# run rosdep
RUN . ${SOURCE} && \
    rosdep init || true && \
    rosdep update || true && \
    rosdep install --from-paths src --ignore-src -r -y || echo "rosdep install completed with warnings"

WORKDIR ${WORKDIR}/src/trac_ik/nlopt
RUN bash ./install_nlopt.sh

WORKDIR ${WORKDIR}
RUN . ${SOURCE} && colcon build --symlink-install

# add sources to .bashrc
RUN echo "source ${SOURCE}" >> ~/.bashrc && \
    echo "if [ -f ${WORKDIR}/install/setup.bash ]; then source ${WORKDIR}/install/setup.bash; fi" >> ~/.bashrc

# Default command
CMD ["bash"]
