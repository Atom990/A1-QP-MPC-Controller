# VeRT-Ctrl

## Gazebo Simulation Setup

Open your terminal and go to the src folder of you catkin workspace, and then:

```
git clone https://github.com/Atom990/VeRT-Sim.git
cd ..
catkin build
```

If you build the workspace sucessfully, you can use this command to open the simulation environment:

```
roslaunch stair_climbing_sim gazebo.launch
```

We also provide some convenient command for tunning.

If you want to make the robot go back to its default configuration and pose, try:

```
rosrun unitree_controller unitree_servo # all the joints should move to its default position
rosrun stair_climbing_sim move_to_default_pos # now the robot will move back to its default pose
```

## Controller Setup

We use docker to setup our control program. We assume you have already installed Docker on your computer.

Open your terminal, and try:

```
git clone https://github.com/Atom990/VeRT-Docker.git
cd VeRT-Docker/docker
docker build -t vert_ctrl_image .
```

After the image is built sucessfully, try:

```
docker run -d \
--network host \
--cap-add=IPC_LOCK --cap-add=sys_nice \
-v PATH_OF_THE_REPO_ON_YOUR_HOST_COMPUTER:/root/VeRT-Ctrl \
--device /dev/input \
--name vert_ctrl_docker \
vert_ctrl_image
```

Now the Docker container should have been started. Try these commands to open the container and build our controller:

```
docker exec -it vert_ctrl_docker bash
cd ~/VeRT-Ctrl
catkin build
```

If everything is good now, the catkin workspace should be built sucessfully.

Now, plug in your joystick, such as an Xbox Controller. Run this command to start joy_node

```
rosrun joy joy_node
```

Then launch the Gazebo simulation environment and move all the joints and the robot itself to default pose. Finally, we can start the controller and use joystick to control the robot:

```
cd ~/VeRT-Ctrl
source devel/setup.bash # you can add this line to .bashrc file
roslaunch stair_climbing_sim_ctrl a1_ctrl.launch
```