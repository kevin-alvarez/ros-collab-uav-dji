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

## Libraries (Python 3)
* Flask 1.0.2
* Flask SocketIO 4.2.1

## Helpers libraries (Python 3)
### Qr
* qrcode 6.1
* matplotlib 3.1.1

