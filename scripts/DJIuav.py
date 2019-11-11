#!/usr/bin/env python3

import rospy
import threading
import multiprocessing
import sys
import os
from std_msgs.msg import String, UInt8
from flask import Flask
from flask_socketio import SocketIO, emit

class DJIuav:

  def __init__(self, uav_id):
    self.loop_rate = 10 # Message frequency (in hz)
    threading.Thread(target=lambda: rospy.init_node('DJIuav'+uav_id, anonymous=False, disable_signals=True)).start()

    # Init variables
    self.uav_id = uav_id
    self.mission_waypoints = ""

    # Suscriber definition
    rospy.Subscriber('DJIuav'+uav_id+'/missionwaypoints', String, self.recv_mission_waypoints)

    # Publishers definitions
    self.pub_mission_progress = rospy.Publisher('DJIuav'+uav_id+'/missionprogress', UInt8, queue_size=10)
    self.pub_camera_status = rospy.Publisher('DJIuav'+uav_id+'/camerastatus', UInt8, queue_size=10)
    self.pub_motor_status = rospy.Publisher('DJIuav'+uav_id+'/motorstatus', UInt8, queue_size=10)
    self.pub_battery_level = rospy.Publisher('DJIuav'+uav_id+'/batterylevel', UInt8, queue_size=10)

    rospy.loginfo("Node DJIuav "+uav_id+" ready...")



  def recv_mission_waypoints(self, data):
    # transformar string de waypoints a lista...
    self.mission_waypoints = data.data

  def publish_data(self, data):
    data = data.split(',')
    mission_progress = int(data[1])
    camera_status = int(data[2])
    motor_status = int(data[3])
    battery_level = int(data[4])
    self.pub_mission_progress.publish(mission_progress)
    self.pub_camera_status.publish(camera_status)
    self.pub_motor_status.publish(motor_status)
    self.pub_battery_level.publish(battery_level)

  def get_uav_id(self):
    return self.uav_id

  def run(self):
    # status variables definition
    mission_progress = 50
    camera_status = 0
    motor_status = 0
    battery_level = 30
    # ROS loop def
    rate = rospy.Rate(self.loop_rate)
    while not rospy.is_shutdown():
      self.pub_mission_progress.publish(mission_progress)
      self.pub_camera_status.publish(camera_status)
      self.pub_motor_status.publish(motor_status)
      self.pub_battery_level.publish(battery_level)
      rospy.loginfo("Mission waypoints: [{}]".format(self.mission_waypoints))
      rate.sleep()

# Flask directives
app = Flask(__name__)
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
  print("A node was disconnected...")

@socketio.on('uav_data')
def uav_data(data):
  print("Data was sended from client: [{}]".format(data))
  uav_node.publish_data(data)


if __name__ == '__main__':
  try:
    flask_node_port = 5000 + int(sys.argv[1])
    socketio.run(app, host=os.environ['ROS_IP'], port=flask_node_port)
  except rospy.ROSInterruptException:
    pass
