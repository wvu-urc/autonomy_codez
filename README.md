# Autonomy Mission Code
Nate Adkins

## Interacting with and running code

This starts the controller node

```bash
ros2 run autonomy_pkg autonomy_ptp_planner_node
```
This starts the text interface node

```bash
ros2 run autonomy_pkg autonomy_text_interface_node
```
<br>
<br>
<br>

This does both

```bash
ros2 launch autonomy_pkg autonomy.launch.py
```

## Structure

- ### [helpers.py](src/autonomy/autonomy_pkg/autonomy_pkg/helpers.py): helper functions for performing relevant operations
- ### [interface.py](src/autonomy/autonomy_pkg/autonomy_pkg/interface.py): text interface for sending goal points while testing
- ### [mgmt.py](src/autonomy/autonomy_pkg/autonomy_pkg/mgmt.py): higher order queue manager and search manager, long term will send points, queue points, and send signals to enable autonomy lights
- ### [pid.py](src/autonomy/autonomy_pkg/autonomy_pkg/pid.py): contains a basic pid controller
- ### [planner.py](src/autonomy/autonomy_pkg/autonomy_pkg/planner.py): ros wrapper for controller-ing robot heading and distance errors (do the thing, get to things)


## Misc. Notes:
- the contents of the [config](src/autonomy/autonomy_pkg/config) directory is not actively used, will be for later 
- the [autonomy_interfaces](src/autonomy/autonomy_interfaces) directory is if custom messages are needed in a cpp-buildable package
- [TODO.md](src/autonomy/TODO.md) contains condensed mission rules, and things to still be done 

## Known Limitations
This autonmy stack is lacking in many areas that we did not have time to finish.
The general idea is a list of points are provided to the rover, the rover uses PID control to traverse between these points in a straight line, in order
There is no object detection or avoidance implemented. For large obstacles, these were visible on our MDRS slope map. Smaller obstacles were handled by the design of our whegs which can climb over very rocky terrain.
If an aruco marker is specified, the rover will traverse to it without regard to the environment if it sees it, even if the next few waypoints are not aruco. This means that paths must made very specifically to avoid large obstacles and position the rover just right before specifying which marker to use.
If the GUI is used in conjunction with the autonomy stack, it must be restarted after each Aruco Point is reached. The cause of this issue is not known at this time