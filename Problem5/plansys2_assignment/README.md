# PlanSys2 Automated Planning Assignment

## Description
Folder structure taken from: https://github.com/PlanSys2/ros2_planning_system_examples/tree/rolling/plansys2_simple_example

## How to build

```
colcon build --symlink-install
rosdep install --from-paths src --ignore-src -r -y      # change src path if needed 
colcon build --symlink-install

source /root/plansys2_ws/install/setup.bash     # change path if needed
```

## How to run

In terminal 1:

```
ros2 launch plansys2_assignment plansys2_assignment_launch.py
```

In terminal 2:

```
ros2 run plansys2_terminal plansys2_terminal        # enters in PlanSys2 Terminal


( ... copy & paste ./launch/commands  or  source .... launch/commands ...)


get plan                                              # Creates plan and shows it
run                                                   # Creates plan and runs it
```
