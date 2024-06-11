import time, sys
import mujoco
import mujoco.viewer
import numpy as np
from utils.createXML import create_MJCF, create_obstacles, generate_from_file


# Rough estimate of length 30 * 0.03

custom_obstacles = generate_from_file("./utils/taskspace.conf")
#custom_obstacles = None

enable_movement = True
#obstacles = create_obstacles('./MJCFS/cylinder_obstacle.xml', 3, pos=[(0.5, -0.15, 0), (0.5, 0.15, 0), (1.1, 0.15, 0)])
create_MJCF(30, 0.05, extra=custom_obstacles, movement=enable_movement, destination='./MJCFS/new_cont.xml')
m = mujoco.MjModel.from_xml_path('./MJCFS/new_cont.xml')
#m = mujoco.MjModel.from_xml_path('test.xml')
#m = mujoco.MjModel.from_xml_path('./MJCFS/cylinder_obstacle.xml')
d = mujoco.MjData(m)
mujoco.mj_resetData(m, d)


ctrl_limit = 3
ctrl_interpolation = 0.0005
direction = 0


velocity = 0
vel_delta = 0.2



apply_torque = True
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
    
    if chr(keycode) in chr(263):
      velocity -= vel_delta
    elif chr(keycode) == chr(262):
      velocity += vel_delta
    elif chr(keycode) == chr(264):
      velocity = 0
    print(f"Velocity: {velocity}")



#print(dir(mujoco.viewer))
runtime_speed = 1
with mujoco.viewer.launch_passive(m, d, key_callback=key_callback, show_left_ui=False) as viewer:

  start = time.time()
  while viewer.is_running() and time.time() - start < 1000: # 1000 seconds
    step_start = time.time()

    mujoco.mj_step(m, d)

    if viewer.cam.distance <= 3.5:
      viewer.cam.distance += 0.002
      viewer.cam.azimuth += 0.1
      viewer.cam.lookat[0] += 0.002
        
        
    if apply_torque:
      #d.ctrl += ctrl_interpolation * direction * pause_torque
      for idx, actuator in enumerate(d.ctrl): # assumes sphere movement is the first joint, and only slide joint
          if idx == 0:
            d.ctrl[idx] = velocity
          else:
            if abs(actuator + ctrl_interpolation * direction) <= ctrl_limit:
              d.ctrl[idx] += ctrl_interpolation * direction
      # if abs(d.ctrl[-1]) >= ctrl_limit:
      #   #direction *= -1 
      #   pause_torque = 0
    
    

    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)


    viewer.sync()

    # Remove to run as fast as possible!
    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step =runtime_speed * (m.opt.timestep - (time.time() - step_start))
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)