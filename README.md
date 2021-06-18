## Analysis and Simulation of Robotic Arms: SCARA & UR5 
Final Year B.Tech. Project 08/2020 - 04/2021

### Objectives
- To first get acquainted with the fundamentals concepts and then complete the analysis and control of both the robotic arms: SCARA and UR5
- To use tools like MATLAB/Simulink, ROS and Gazebo for simulation purposes
- To perform different applications like pick and place, straight line motion and others in simulation
- To give readers a overview of the theoretical analysis of the two robotic arms and capabilities of the simulation softwares, along with references

Project report contains all the information regarding the project.

This repository contains the ROS packages for both the arms. MATLAB simulation files are not included in this repository.

### Prerequisites
- ROS
- Gazebo
- MoveIt!
- FindObject2d

The packages can be built using catkin_make.

### SCARA simulation

- 'H' character writing [Video link](https://youtu.be/aVlcPpxdjcQ)  
To run

```bash
roslaunch epson_g3_moveit_new scara.launch
```

```bash
rosrun epson_g3_moveit_new display_3d.py 
```

```bash
rosrun epson_g3_moveit_new character.py
```

### UR5 simulation

- Picking up a rolling ball [Video link](https://youtu.be/BM52mtVZ3e8)  
To run

```bash
roslaunch ur5_prj ur5_ly_prj.launch
```

```bash
rosrun ur5_prj moveit_new.py
```