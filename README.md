# laser_autonomous_exploration
Autonomous exploration of Laser-Robotics Lab at UFPB using Turtlebot3 and L1BR

---

## Requirements

* Docker (and docker-compose): recommended to install [Docker Desktop](https://www.docker.com/products/docker-desktop/)

---

## First-time setup

To run Docker without `sudo`:

```bash
sudo groupadd docker
sudo gpasswd -a $USER docker
newgrp docker
```

Make sure your docker is initialized before proceeding and your nvidia driver is installed.

### Setting up NVIDIA Container Toolkit
Setup the package repository and the GPG key:
```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

Install the nvidia-container-toolkit package (and dependencies) after updating the package listing:
```bash
sudo apt-get update
```

```bash
sudo apt-get install -y nvidia-container-toolkit
```

Configure the Docker daemon to recognize the NVIDIA Container Runtime:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
```

Restart the Docker daemon to complete the installation after setting the default runtime:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl daemon-reload && sudo systemctl restart docker
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

At this point, a working setup can be tested by running a base CUDA container:
```bash
sudo docker run --rm --runtime=nvidia --gpus all nvidia/cuda:11.6.2-base-ubuntu20.04 nvidia-smi
```

# Docker with GUIs

If you've done the first-time setup, remember to do the following on every system reboot:

- If on **Linux**, remember to run 
    ```bash
    xhost +local:docker && sudo docker compose up
    ```
    on every reboot.

# Running the project

Spin-up the containers with:

```bash
docker-compose up
```

If this is your first time here, it might take a couple minutes to build the image. Once it's done, you should see `explore`, `gazebo`, and `rostcp` containers up and their messages.

To shutdown, use `CTRL+C` in the terminal running the containers.

**NOTE:** often, the robot gets stuck at the start of the simulation. To fix this, go to Rviz and give it a `Nav2 Pose Goal` (make the robot move and map a bit). If the `explore` node considered the exploration done at the start of the simulation, follow the instruction in [Resume exploration](#resume-exploration).

## Detached mode

Alternatively, you can run the containers in detached mode:

```bash
docker-compose up -d
```

This will leave the terminal free while the containers run in the background. To shutdown, run the following:

```bash
docker-compose down
```

## Opening a new terminal inside a container

You can `docker exec` into any of the containers and run `ros2` commands from the get-go (sourcing is done automatically). For example, going into the `explore` container:

```bash
docker exec -it explore bash
```

## Resume exploration

Sometimes the `explore` node will stop exploration, reporting that there are no more frontiers. This can happen when the simulation takes too long to launch. To resume exploration, `exec` into a container and run the following command:

```bash
ros2 topic pub /explore/resume std_msgs/Bool '{data: true}' -1
```

This publishes a single message to the `/explore/resume` topic, toggling the exploration back on. If the exploration keeps stopping, remove the `-1` so it is constantly resumed.

```
