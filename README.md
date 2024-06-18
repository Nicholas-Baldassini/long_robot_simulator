# long_robot_simulator
Welcome to the long robot simulator repo. In this repo you will find a continuum framework built on top of the simulation engine Mujoco.

## Getting started
Currently this has only been tested on linux but should work on all platforms. If you encounter any issues please leave an issue on this github repo and I will assist as soon as possible.

1. Clone this repo
2. Install python

    2. On linux you must install tkinter seperately ```sudo apt-get install python3-tk```
3. Optional: create a virtualenv
```
virtualenv mujEnv
source mujEnv/bin/activate
```
4. ```pip install -r requirments.txt```

This should be it. Do let me know if there are any issues.

## Usage
To run the simulation,
```python main.py```

#### Actuation
Right now the robot controls are quite simple. The robot starts off without any actuation and you can click ```spacebar``` to start actuation. Click it again will change the direction. 

#### Movement
In main.py there is a boolean ```enable_movement = True```, if False the base of the robot will be locked in place. If True, you can move the robot in the y direction with the arrow keys and your velocity will be printed in the console. The movement currently is a little finicky and we are working on fixing it.


#### Main control loop
In main.py you will find the ```while``` loop where the simulation takes place. The only real important part is the,
```
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
```
This is where the joint/torque control resides. Right now it is only controlled by the keyboard but feel free to play around with it.


## Creating a taskspace
```
cd utils
python createTaskspace.py
```
You will be shown a GUI. Click where you want the obstacles to be, you can change sizes as well as toggle static or non static obstacles. Then click "save to file" then you can ```cd ..``` and do python main.py. Do note if you save to file your previous taskspaces will be deleted.



## Features
If you encounter any bugs or issues with the repo let me know in the issue tab on github. Also if you have any ideas for new features you would like to see, leave an issue as well. This project is meant for researchers so your feedback is important. We want to know what would make this model useful for your work. Leave a suggestion and I will look into it and possibly impliment it.





enjoy long robots! Thanks.