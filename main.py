import time
import mujoco
import mujoco.viewer
from createXML import create_MJCF


create_MJCF(30, 0.05)
m = mujoco.MjModel.from_xml_path('./new_cont.xml')
d = mujoco.MjData(m)
#import pdb; pdb.set_trace()

mujoco.mj_resetData(m, d)
ctrl_limit = 1
ctrl_interpolation = 0.001
direction = -1

with mujoco.viewer.launch_passive(m, d) as viewer:
  start = time.time()
  while viewer.is_running() and time.time() - start < 120:
    step_start = time.time()


    mujoco.mj_step(m, d)
    
    #print(abs(d.ctrl[-1]), direction)
    d.ctrl += ctrl_interpolation * direction
    if abs(d.ctrl[-1]) >= ctrl_limit:
      direction *= -1 
    

    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)


    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)