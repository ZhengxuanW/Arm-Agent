FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-moveit \
    ros-humble-moveit-resources-panda-moveit-config \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /sandbox/ws

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'if [ -f /sandbox/ws/install/setup.bash ]; then source /sandbox/ws/install/setup.bash; fi' >> /root/.bashrc

CMD ["bash"]
