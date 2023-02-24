FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       ros-foxy-rviz2 \
                       python3-colcon-common-extensions
RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d

# ros2 package
RUN mkdir -p ctrl_ws/src/
COPY src/. /ctrl_ws/src
RUN source /opt/ros/foxy/setup.bash && \
    cd ctrl_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro foxy -y && \
    colcon build

WORKDIR '/ctrl_ws'

ENTRYPOINT ["/bin/bash"]
