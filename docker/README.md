## Install nvidia-docker
Follow instructions on this [link](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installing-on-ubuntu-and-debian)

## Build

```bash
docker build --no-cache -t mros_exp:foxy -f docker/Dockerfile .
```

## RUN

```bash
xhost +local:root
docker run --rm -it -e DISPLAY=$DISPLAY -v="/tmp/.gazebo/:/root/.gazebo/" -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all mros_exp:foxy ros2 launch navigation_experiments_mc_bts_pddl_base tb3_sim_launch.py headless:=False
```

```bash
docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all mros_exp:foxy ros2 launch navigation_experiments_mc_bts_pddl_base nav2_turtlebot3_launch.py
```

```bash
docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all mros_exp:foxy ros2 launch navigation_experiments_mc_bts_pddl_base metacontrol.launch.py
```

### BT

```bash
docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all mros_exp:foxy ros2 launch navigation_experiments_mc_bts bt_controller_launch.py
```

### PDDL

```bash
docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all mros_exp:foxy ros2 launch navigation_experiments_mc_pddl pddl_controller_launch.py
```

```bash
docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all mros_exp:foxy ros2 run navigation_experiments_mc_pddl patrolling_controller_node
```
