FROM osrf/ros:humble-desktop

### === INITIAL SETUP === ###

# UNIX username
ENV USER=xplorer

# Add new user and create a home directory for him (attempt at fixing WSL2 lag)
RUN useradd --create-home --shell /bin/bash ${USER}

# Set working directory
ENV WORKDIR=/home/${USER}/src/
WORKDIR ${WORKDIR}

# ROS distribution
ENV ROS_DISTRO=humble


### === INSTALLS === ###

# Update apt so that new packages can be installed properly. wget for gazebo, dos2unix for line endings fix
RUN apt-get update && apt-get install -y wget dos2unix \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*rm 

# Install ROS2-humble packages
RUN sudo apt-get install ros-humble-rosbridge-suite -y \
    && sudo apt-get install ros-humble-turtlebot3* -y \
    && sudo apt-get install ros-humble-navigation2 -y \
    && sudo apt-get install ros-humble-nav2-bringup -y \
    && sudo apt -y install ros-humble-gazebo-ros-pkgs -y \
    && sudo apt -y install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control -y 

RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

# Install gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
    && apt-get update -y && apt-get install -y gazebo11 libgazebo11-dev

# Copy and build ROS2 packages inside the workspace
RUN mkdir ./xplorer_ws/src -p
COPY xplorer_ws/src ./xplorer_ws/src
RUN cd ./xplorer_ws && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r --rosdistro humble -y && \
    colcon build


### === FINAL SETUP === ###

# Copy gazebo worlds and models
# COPY ./worlds /opt/ros/humble/share/turtlebot3_gazebo/worlds/
# COPY ./models /opt/ros/humble/share/turtlebot3_gazebo/models/

# Replace ros_entrypoint.sh
COPY ros_entrypoint.sh /ros_entrypoint.sh

# Sources ROS on every new terminal automatically
RUN echo '. /opt/ros/$ROS_DISTRO/setup.sh' >> ~/.bashrc && \
    echo '. ${WORKDIR}/xplorer_ws/install/setup.bash' >> ~/.bashrc


### === CLEANUP === ###

# Use dos2unix to convert the line endings, remove dos2unix and wget, then clean up files created by apt-get
RUN dos2unix /ros_entrypoint.sh && apt-get --purge remove -y dos2unix wget && rm -rf /var/lib/apt/lists/*

# Required in Linux systems. Gives proper permissions to entrypoint file.
RUN ["chmod", "+x", "/ros_entrypoint.sh"]