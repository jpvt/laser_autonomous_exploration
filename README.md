# laser_autonomous_exploration
Autonomous exploration of Laser-Robotics Lab at UFPB using Turtlebot3 and L1BR

---

## Requirements

* Docker (and docker-compose): recommended to install Docker Desktop

---

## First-time setup

To run Docker without `sudo`:

```bash
sudo groupadd docker
sudo gpasswd -a $USER docker
newgrp docker
```

Run the following command:

```bash
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' gazebo`
```

**IMPORTANT:** This command is required on every reboot.

In the `environment.env` file, check if `DISPLAY` is correct by opening a terminal and running:

```bash
echo $DISPLAY
```

Write the result to `DISPLAY` (normally, it's either `:0` or `:1`). Now you're ready to run the project!

**Note:** If the project still doesn't work, you might need to install [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit), then run the following:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

# Docker with GUIs

If you've done the first-time setup, remember to do the following on every system reboot:

- If on **Linux**, remember to run 
    ```bash
    xhost +local:`docker inspect --format='{{ .Config.Hostname }}' gazebo`
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

## Changing the Gazebo world

The map is set by default to `labyrinthe.world`, a complex labyrinthe world created for this project. It is separated in 4 zones, with a central start area:
- red: longer hallways and a spiral
- blue: simple long hallways, a path with many tight corners, and a trident shaped path
- green: furniture room with 2 stairs, a series of 3 small tables, and a big table
- purple: highly chaotic and randomly placed walls

All zones are connected to their neighbor zones and the central area.

![alt text](./docs/labyrinthe-model.jpeg "Labyrinthe world")

To change the world loaded in Gazebo, open `docker-compose.yml` and look for the `gazebo` service. There, under the `command`, change the last part of the path in the `world:=` parameter with one of the following (or any worlds added to the `worlds` folder):

- labyrinthe.world
- empty_world.world
- turtlebot3_world.world
- turtlebot3_house.world
- turtlebot3_dqn_stage1.world
- turtlebot3_dqn_stage2.world
- turtlebot3_dqn_stage3.world
- turtlebot3_dqn_stage4.world

For example, to change it to the `turtlebot3_house.world` world, the final command would look like this:

```docker
command: ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True world:=/opt/ros/galactic/share/turtlebot3_gazebo/worlds/turtlebot3_world.world
```

**NOTE:** except for the labyrinthe, this will load the world without spawning the robot. To add a robot, go to the `Insert` tab and add a `turtlebot` to the world.

**NOTE2:** the labyrinth is intentionally called labyrinthe, as it was made in France.

## Creating new worlds

The `gazebo` container can be used to create new worlds and models. Follow the following steps:
1. Set one of the `.world` worlds as described in [Changing the Gazebo map](#changing-the-gazebo-map) to use it as a base/template and spin-up the `gazebo` container
2. In the Gazebo simulation, click on the `Edit` tab in the toolbar, then `Building editor`
3. Create your model (warning: you can't save and edit it later!)
4. Save the model somewhere easy to find in the container file system (e.g., the `root` folder)
5. (requires the `Docker` extension on VS Code) Go to VSCode, access the Docker tab, and search for the model file you saved. Download it to the `models` folder
6. Exit the `Building editor`
7. Go to the `Insert` tab and click on `Add Path`. Search for the folder containing your model's folder and add it
8. Now you can add your model to the world. Add any other models as desired.
9. Once done, go to `File` and `Save world as`. Save it in an easy-to-find folder (e.g., root) as a `.world` file
10. Repeat step `5.`, but for the `.world` file, and save it in the `worlds` folder.

With this, your world is available for use by following the [Changing the Gazebo map](#changing-the-gazebo-map) subsection. All models saved to the `models` folder will also be available in the container next to the `turtlebot` models.

**NOTE:** new files in the `models` and `worlds` folders will require the container to be rebuilt with:

```bash
docker-compose up --build
```
