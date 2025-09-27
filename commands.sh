source devel/setup.bash
roslaunch ow_reemc gazebo.launch



source devel/setup.bash
roslaunch ow_reemc sim.launch



source devel/setup.bash

# Step in place.
rosservice call /open_walker/plan_fixed_stamp

# Walk in a straight line.
rosservice call /open_walker/plan_fixed_line

# Walk to the side
rosservice call /open_walker/plan_fixed_sidestep_left
rosservice call /open_walker/plan_fixed_sidestep_right

# Walk in circles.
rosservice call /open_walker/plan_fixed_circle_right
rosservice call /open_walker/plan_fixed_circle_left

# Generic plan with 20 steps of 0.2 m length and 0.1 rad angle 
rosservice call /open_walker/plan_fixed "n_steps:
  data: 20
length:
  data: 0.2 
lateral:
  data: 0.0
angle:
  data: 0.1"