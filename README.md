# TAMS KUKA LWR
![kuka](doc/kuka_fig.jpg)
## Introduction
- our setup uses the FRI (fast-research-interface) for real-time communication with the arm controller at 100Hz+. 
- the ROS node is an improved version of the `FRILibrary` code from Torsten Kröger and the `Reflexxes-TypeII` real-time motion generation, providing joint-position goals, velocity goals, trajectory execution, and both joint and cartesian compliance modes. See the `ros_fri` package for details 
- startup of the robot is a bit fiddly, make sure to select the correct tool and coordinate base system at startup, and see below for detailed instructions and tips [doc here](doc/kuka_quick_start_guide/quick_start_kuka.md).
- once the KUKA controller detects a motion anomaly it basically locks up. Restart the robot and the ROS nodes.

## ROS workspace setup
- In your workspace
```
git clone https://github.com/lianghongzhuo/reflexxes_type2.git
git clone https://github.com/lianghongzhuo/frilibrary.git
git clone https://github.com/TAMS-Group/tams_apriltags.git
git clnoe git@git.crossmodal-learning.org:TAMS/tams_lwr.git
```

- Build your workspace with
`catkin build`

## Robot setup
- see [doc here](doc/kuka_quick_start_guide/quick_start_kuka.md) for a step to step instruction.

## Run robot with ROS
- see [doc here](doc/run_lwr_with_ros.md) for a step to step instruction.

## PDF documentations
- All pdf are at: https://git.crossmodal-learning.org/TAMS/tams_lwr_doc
