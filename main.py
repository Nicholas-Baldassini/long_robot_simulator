import time
import mujoco
import mujoco.viewer
import numpy as np
from createXML import create_MJCF, create_obstacles


# Rough estimate of length 30 * 0.03

obstacles = create_obstacles('./MJCFS/cylinder_obstacle.xml', 3, pos=[(0.5, -0.15, 0), (0.5, 0.15, 0), (1.1, 0.15, 0)   ])
create_MJCF(30, 0.07, extra=obstacles, destination='./MJCFS/new_cont.xml')
m = mujoco.MjModel.from_xml_path('./MJCFS/new_cont.xml')
#m = mujoco.MjModel.from_xml_path('test.xml')
#m = mujoco.MjModel.from_xml_path('./MJCFS/cylinder_obstacle.xml')
d = mujoco.MjData(m)


mujoco.mj_resetData(m, d)
ctrl_limit = 4
ctrl_interpolation = 0.001
#ctrl_interpolation = 0
direction = -1



paused = False
def key_callback(keycode):
  if chr(keycode) == ' ':
    global paused
    paused = not paused
    
with mujoco.viewer.launch_passive(m, d, key_callback=key_callback, show_left_ui=False) as viewer:
  start = time.time()
  while viewer.is_running() and time.time() - start < 240:
    step_start = time.time()

    if not paused:
      mujoco.mj_step(m, d)
      
      #print(abs(d.ctrl[-1]), direction)

      
      d.ctrl += ctrl_interpolation * direction
      if abs(d.ctrl[-1]) >= ctrl_limit:
        direction *= -1 
        #direction = 0
      

      with viewer.lock():
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)


      viewer.sync()

      # Rudimentary time keeping, will drift relative to wall clock.
      time_until_next_step = m.opt.timestep - (time.time() - step_start)
      if time_until_next_step > 0:
        time.sleep(time_until_next_step)