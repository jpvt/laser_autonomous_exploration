ARG ROS_DISTRO=humble
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

ARG PREFIX
ENV USER=xplorer

# Add new user and create a home directory for him (attempt at fixing WSL2 lag)
RUN useradd --create-home --shell /bin/bash ${USER}

# Set working directory
ENV WORKDIR=/home/${USER}/src/
WORKDIR ${WORKDIR}

# select bash as default shell
SHELL ["/bin/bash", "-c"]

ENV IGNITION_VERSION fortress
ENV HUSARION_ROS_BUILD simulation

#WORKDIR /xplorer_ws

# Copy files to container
RUN mkdir ./xplorer_ws/src -p
COPY xplorer_ws/src ./xplorer_ws/src

# Update apt so that new packages can be installed properly. wget for gazebo, dos2unix for line endings fix
# install everything needed
RUN MYDISTRO=${PREFIX:-ros}; MYDISTRO=${MYDISTRO//-/} && \
    apt-get update --fix-missing && apt-get install -y \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-$ROS_DISTRO-teleop-twist-keyboard \
        ros-$ROS_DISTRO-twist-mux \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control \
        git && \
    apt-get upgrade -y && \
    cd ./xplorer_ws && \
     . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths . --ignore-src -r --rosdistro $ROS_DISTRO -y && \
    colcon build && \
    # make the image smaller
    export SUDO_FORCE_REMOVE=yes && \
    apt-get remove -y \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        git && \   
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Replace ros_entrypoint.sh
COPY ros_entrypoint.sh /ros_entrypoint.sh

# Sources ROS on every new terminal automatically
RUN echo '. /opt/ros/$ROS_DISTRO/setup.sh' >> ~/.bashrc && \
    echo '. ${WORKDIR}/xplorer_ws/install/setup.bash' >> ~/.bashrc

RUN ["chmod", "+x", "/ros_entrypoint.sh"]


RUN echo $(cat /xplorer_ws/src/ugv_simulation/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') > /version.txt
