# Extract and place the folder in home

cd ~/reemc_docker
docker build -t reemc .

# After building run the image
docker run -it --gpus all --rm --user ros --network=host --ipc=host -e DISPLAY=$DISPLAY -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all -v /tmp/.X11-unix:/tmp/.X11-unix:rw reemc:latest



# This will enter into container

# Terminal - 1
source /home/ros/reemc_ws/devel/setup.bash
roslaunch ow_reemc gazebo.launch


# Terminal - 2
source /home/ros/reemc_ws/devel/setup.bash
roslaunch ow_reemc sim.launch


# Terminal - 3
source /home/ros/reemc_ws/devel/setup.bash

# Step in place.
rosservice call /reemc_walker/plan_fixed_stamp

# Walk in a straight line.
rosservice call /reemc_walker/plan_fixed_line

# Walk to the side
rosservice call /reemc_walker/plan_fixed_sidestep_left
rosservice call /reemc_walker/plan_fixed_sidestep_right

# Walk in circles.
rosservice call /reemc_walker/plan_fixed_circle_right
rosservice call /reemc_walker/plan_fixed_circle_left

# Generic plan with 20 steps of 0.2 m length and 0.1 rad angle 
rosservice call /reemc_walker/plan_fixed "n_steps:
  data: 20
length:
  data: 0.2 
lateral:
  data: 0.0
angle:
  data: 0.1"







