# **LASER Autonomous Exploration**

![Working Demo](examples/working_demo_1.gif)

Welcome to the LASER Autonomous Exploration repository! This project aims to develop an efficient and robust exploration and mapping system for the L1BR robot, a custom differential drive robot designed by LASER-Robotics Lab. The system uses the SLAM Toolbox for simultaneous localization and mapping (SLAM), Nav2 for navigation, and Explore Lite for autonomous exploration.

---

## **Overview**

We aim to minimize the time it takes for the L1BR robot to explore and map unknown environments. We achieve this by integrating state-of-the-art ROS2 packages and optimizing the exploration strategy. The main components of our project are:

* **SLAM Node:** Utilizes SLAM Toolbox for mapping the environment and simultaneously estimating the robot's pose.
* **Navigation Node:** Implements Nav2 for path planning and path following, allowing the robot to navigate safely and efficiently through the environment.
* **Exploration Node:** Uses Explore Lite to drive the exploration strategy, enabling the robot to find and navigate to unexplored regions.

---

## **Getting Started**

### **Prerequisites**

* **Docker and Docker-Compose:** 
  * Install [Docker Desktop](https://www.docker.com/products/docker-desktop/)
  * To run Docker without `sudo`:
    ```bash
    sudo groupadd docker
    sudo gpasswd -a $USER docker
    newgrp docker
    ```
  * Make sure your docker is initialized and your nvidia driver is installed before proceeding.


* **NVIDIA Container Toolkit:** (only for Nvidia GPU):
  * **Setting up NVIDIA Container Toolkit:** Setup the package repository and the GPG key.
    ```bash
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
        && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
        && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
                sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
                sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    ```
  * Install the nvidia-container-toolkit package (and dependencies) after updating the package listing:
    ```bash
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    ```
  * Configure the Docker daemon to recognize the NVIDIA Container Runtime:
    ```bash
    sudo nvidia-ctk runtime configure --runtime=docker
    ```
  * Restart the Docker daemon to complete the installation after setting the default runtime:
    ```bash
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl daemon-reload && sudo systemctl restart docker
    sudo apt-get update
    sudo apt-get install -y nvidia-docker2
    sudo systemctl restart docker
    ```
  * At this point, a working setup can be tested by running a base CUDA container:
    ```bash
    sudo docker run --rm --runtime=nvidia --gpus all nvidia/cuda:11.6.2-base-ubuntu20.04 nvidia-smi
    ```

### **Accessing Maps from the Container’s Volume**
The generated maps from the simulation are saved to the `/maps` directory inside the `laser_l1br` container, which is mounted to the `maps` volume on the host machine. This allows you to persist the generated maps even after the container is stopped or removed.

To access the maps from the container’s volume, you can use the `docker cp` command to copy the map files from the container to a directory on the host machine. For example, to copy all of the map files from the `/maps` directory inside the container to a directory called `my_maps` on the host machine, you can run the following command:

```docker cp laser_l1br:/maps/. my_maps/```

After running this command, you will be able to access the generated map files from the `my_maps` directory on your host machine.

### **Installation**

**Note:** The current build is under development and it was only tested for Linux. In Windows, you may need to install other prerequisites. (Check the [Xplorer repo](https://github.com/Fabulani/xplorer) for some instructions on installing for windows)

1. Clone this repository to your local machine.
2. Navigate to the project root folder and bring up the Docker Compose:
   - If on **Linux**, remember to run (on every reboot):
    ```bash
    xhost +local:docker && sudo docker compose up
    ```

If this is your first time here, it might take a couple minutes to build the image. Once it's done, you should see `laser_l1br` container up and it's messages.

To shutdown, use `CTRL+C` in the terminal running the containers.

**NOTE:** often, the robot gets stuck at the start of the simulation (Depending on the initial position). To fix this, go to Rviz and give it a `Nav2 Pose Goal` (make the robot move and map a bit). If the `explore` node considered the exploration done at the start of the simulation, follow the instruction in **Resume exploration**.

### **Operating Environment**
**Detached mode**

Alternatively, you can run the containers in detached mode:

```bash
xhost +local:docker && sudo docker compose up -d
```

This will leave the terminal free while the containers run in the background. To shutdown, run the following:

```bash
sudo docker compose down
```

**Opening a new terminal inside a container**

You can `docker exec` into any of the containers and run `ros2` commands from the get-go (sourcing is done automatically). For example, going into the `explore` container:

```bash
sudo docker exec -it laser_l1br bash
```

**Resume exploration**

Sometimes the `explore` node will stop exploration, reporting that there are no more frontiers. This can happen when the simulation takes too long to launch. To resume exploration, `exec` into a container and run the following command:

```bash
ros2 topic pub /explore/resume std_msgs/Bool '{data: true}' -1
```

This publishes a single message to the `/explore/resume` topic, toggling the exploration back on. If the exploration keeps stopping, remove the `-1` so it is constantly resumed.

---

## **Acknowledgments**

* [Gazebo and L1BR Simulation](https://github.com/LASER-Robotics/laser_ugv_system) by LASER-Robotics Lab Team
* [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) by Steve Macenski
* [Navigation2](https://navigation.ros.org/) by the ROS2 Navigation Working Group
* [Explore Lite](https://github.com/robo-friends/m-explore-ros2) by Carlos Alvarez
  