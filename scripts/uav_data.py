#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys

class uav_data:

  def __init__(self, uav_id):
    self.loop_rate = 10 # Message frequency (in hz)
    rospy.init_node('uav_data'+uav_id, anonymous=False)

    # Init variables
    self.mission_data = ""
    self.camera_data = ""
    self.motor_data = ""

    # Suscribers definition
    rospy.Subscriber('DJIuav'+uav_id+'/missionprogress', String, self.recv_mission_data)
    rospy.Subscriber('DJIuav'+uav_id+'/camerastatus', String, self.recv_camera_data)
    rospy.Subscriber('DJIuav'+uav_id+'/motorstatus', String, self.recv_motor_data)

    # Publisher definition
    self.pub_mission_context = rospy.Publisher('missioncontext', String, queue_size=10)

    rospy.loginfo("Node uav_data "+uav_id+" ready...")

  def recv_mission_data(self, data):
    self.mission_data = data.data

  def recv_camera_data(self, data):
    self.camera_data = data.data

  def recv_motor_data(self, data):
    self.motor_data = data.data

  def run(self):
    rate = rospy.Rate(self.loop_rate)
    while not rospy.is_shutdown():
      # Logs
      self.pub_mission_context.publish("Mission Context Published!")
      #rospy.loginfo(self.mission_data + '---' + self.camera_data + '---' + self.motor_data)
      rate.sleep()
      

if __name__ == '__main__': 
  # Falta manejar errores de ingreso de parametros
  try:
    uavd = uav_data(sys.argv[1])
    uavd.run()
  except rospy.ROSInterruptException:
    pass
