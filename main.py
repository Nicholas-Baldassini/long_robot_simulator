import time, sys
import mujoco
import mujoco.viewer
import numpy as np
from utils.createXML import create_MJCF, create_obstacles, generate_from_file

# Choose your colour scheme
colour_scheme = ["Clean", "Fun"]

# Load in your taskspace. Taskspace.conf is generated with utils/createTaskspace.py
custom_obstacles = generate_from_file("./utils/taskspace.conf", colour_scheme=colour_scheme[1])

# Uncomment this if you want NO obstacles in your scene
#custom_obstacles = None 

# If False, the robot base is locked in place
enable_movement = False
#obstacles = create_obstacles('./MJCFS/cylinder_obstacle.xml', 3, pos=[(0.5, -0.15, 0), (0.5, 0.15, 0), (1.1, 0.15, 0)])
create_MJCF(25, 0.05, extra=custom_obstacles, movement=enable_movement, destination='./MJCFS/new_cont.xml', colour_scheme=colour_scheme[1])

# Load in your MJCF file
m = mujoco.MjModel.from_xml_path('./MJCFS/new_cont.xml')
d = mujoco.MjData(m)

# Zero out all data
mujoco.mj_resetData(m, d)

# Torque control limit
ctrl_limit = 2 
# The amount we increase the torque every step, we dont want to instantaneously change it
ctrl_interpolation = 0.001 #0.00002
# Direction of torque, starts off at 0
direction = 0

# For movement
velocity = 0
# Speed increase
vel_delta = 0.2


# Apply torque flag
apply_torque = True

# Keyboard controls
def key_callback(keycode):
  if chr(keycode) == ' ':
    # spacebar, change torque direction
    global direction
    if not direction:
      direction = 1
    direction *= -1  
    print(f"Changing torque direction {direction}")

  else:
    global velocity
    global vel_delta
    # Left arror
    if chr(keycode) in chr(263):
      velocity -= vel_delta
    # Right arrow
    elif chr(keycode) == chr(262):
      velocity += vel_delta
    # Down arrow zeros velocity
    elif chr(keycode) == chr(264):
      velocity = 0
    print(f"Velocity: {velocity}")




cinematics = True
# Change runtime_speed to speed up or slow down simulation
runtime_speed = 1 # 0.20
with mujoco.viewer.launch_passive(m, d, key_callback=key_callback, show_left_ui=False) as viewer:
  start = time.time()
  
  # Main control loop
  while viewer.is_running() and time.time() - start < 1000: # 1000 seconds
    step_start = time.time()

    # Mujoco physics time step
    mujoco.mj_step(m, d)

    # Camera stuff
    if cinematics:
      if viewer.cam.distance <= 3.5:
        viewer.cam.distance += 0.002
        viewer.cam.azimuth += 0.1
        viewer.cam.lookat[0] += 0.002
      else:
        cinematics = False
        
    
    # Apply torque to joints
    if apply_torque:
      for idx, actuator in enumerate(d.ctrl): # assumes sphere movement is the first joint, and only slide joint
          # Skip over first joint. 
          # In this model the first joint is prismatic which alows movement of the robot
          if idx == 0:
            d.ctrl[idx] = velocity
          else:
            if abs(actuator + ctrl_interpolation * direction) <= ctrl_limit:
              d.ctrl[idx] += ctrl_interpolation * direction
    
    

    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)


    viewer.sync()

    # Remove to run as fast as possible!
    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = runtime_speed * (m.opt.timestep - (time.time() - step_start))
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)