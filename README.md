# Collaborative DJI UAV with ROS
ROS package for collaborative tasks with DJI UAV.

## Instructions
Just use ROS launch to run all the nodes with the file named 'collab-uav.launch'.
PD: No need to run roscore, it uploads itself with the launch.

```bash
$ roslaunch collab-uav.launch
```

## Helpful tools
Use *rqt_graph* to see all nodes and topics

```bash
$ rosrun rqt_graph rqt_graph
```

## Libraries (Python 2)
* Flask *version*
* qrcode *version*
* numpy
* matplotlib ?

