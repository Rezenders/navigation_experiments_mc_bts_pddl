## Build

```bash
docker build --no-cache -t mros_exp:foxy -f docker/Dockerfile .
```

## RUN

```bash
xhost +local:root
docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all mros_exp:foxy ros2 launch navigation_experiments_mc_bts_pddl_base tb3_sim_launch.py
```

```bash
docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all mros_exp:foxy ros2 launch navigation_experiments_mc_bts_pddl_base nav2_turtlebot3_launch.py
```

```bash
docker run --rm -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all mros_exp:foxy ros2 launch mros2_reasoner launch_reasoner.launch.py
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
