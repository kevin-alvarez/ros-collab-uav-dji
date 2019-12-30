#!/usr/bin/env python3

import rospy
import threading
import multiprocessing
import sys
import os
from std_msgs.msg import String, UInt8
from flask import Flask
from flask_socketio import SocketIO, emit
import logging

class DJIuav:

  def __init__(self, uav_id):
    self.loop_rate = 10 # Message frequency (in hz)
    threading.Thread(target=lambda: rospy.init_node('DJIuav'+uav_id, anonymous=False, disable_signals=True)).start()

    # Init variables
    self.uav_id = uav_id
    self.new_mission = ""
    self.actual_mission = ""

    # Suscriber definition
    rospy.Subscriber('DJIuav'+uav_id+'/newmission', String, self.recv_new_mission)

    # Publishers definitions
    self.pub_mission_progress = rospy.Publisher('DJIuav'+uav_id+'/missionprogress', UInt8, queue_size=10)
    self.pub_camera_status = rospy.Publisher('DJIuav'+uav_id+'/camerastatus', UInt8, queue_size=10)
    self.pub_motor_status = rospy.Publisher('DJIuav'+uav_id+'/motorstatus', UInt8, queue_size=10)
    self.pub_battery_level = rospy.Publisher('DJIuav'+uav_id+'/batterylevel', UInt8, queue_size=10)
    self.pub_mission_waypoints = rospy.Publisher('DJIuav'+uav_id+'/missionwaypoints', String, queue_size=10)
    self.pub_actual_waypoint = rospy.Publisher('DJIuav'+uav_id+'/actualwaypoint', UInt8, queue_size=10)

    rospy.loginfo("Node DJIuav "+uav_id+" ready...")

  def recv_new_mission(self, data):
    self.new_mission = data.data

  def get_uav_id(self):
    return self.uav_id

# Flask directives
app = Flask(__name__)
log = logging.getLogger('werkzeug')
log.disabled = True # Disable socketio logs
socketio = SocketIO(app, async_mode=None)
uav_node = DJIuav(sys.argv[1])

@app.route("/")
def home():
  return "I'm visible!!"

@socketio.on('connect')
def connect():
  print("A node was connected!!")
  rospy.loginfo("A node was connected to UAV {}".format(uav_node.get_uav_id()))

@socketio.on('disconnect')
def disconnect():
  uav_node.pub_camera_status.publish(0)
  uav_node.pub_motor_status.publish(0)
  uav_node.pub_battery_level.publish(0)
  print("A node was disconnected...")

@socketio.on('camerastatus')
def camera_status(data):
  uav_node.pub_camera_status.publish(int(data))

@socketio.on('motorstatus')
def motor_status(data):
  uav_node.pub_motor_status.publish(int(data))

@socketio.on('batterylevel')
def battery_level(data):
  uav_node.pub_battery_level.publish(int(data))

@socketio.on('missionprogress')
def mission_progress(data):
  progress = round(float(data))
  uav_node.pub_mission_progress.publish(progress)

@socketio.on('missionwaypoints')
def mission_waypoint(data): # Can't simulate with stop, must have own button in app
  uav_node.actual_mission = data
  uav_node.pub_mission_waypoints.publish(data)

@socketio.on('actualwaypoint')
def actual_waypoint(data):
  uav_node.pub_actual_waypoint.publish(int(data))

@socketio.on('requestnewmission')
def request_new_mission():
  if uav_node.actual_mission == uav_node.new_mission: # Si ya se realizo la mision nueva y no ha cambiado
    socketio.emit('newmission', "")
  else:  # Igual puede caer en este punto si la mision es vacia (no existe mision pendiente por UAV vecino)
    socketio.emit('newmission', uav_node.new_mission)

if __name__ == '__main__':
  try:
    flask_node_port = 5000 + int(sys.argv[1])
    socketio.run(app, host=os.environ['ROS_IP'], port=flask_node_port)
  except rospy.ROSInterruptException:
    pass
