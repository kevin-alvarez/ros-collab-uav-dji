#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys

class uav_behaviour:

  def __init__(self, uav_id):
    rospy.init_node('uav_behaviour'+uav_id, anonymous=False)

    # Init variables
    self.mission_context_data = ""

    # Suscriber definition
    rospy.Subscriber('missioncontext', String, self.recv_mission_context)

    # Publisher definition
    self.pub_actual_command = rospy.Publisher('DJIuav'+uav_id+'/actualcommand', String, queue_size=10)

    rospy.loginfo("Node uav_behaviour "+uav_id+" ready...")

  def recv_mission_context(self, data):
    self.mission_context_data = data.data

  def run(self):
    while not rospy.is_shutdown():
      self.pub_actual_command.publish("Follow Mission")

if __name__ == '__main__': 
  # Falta manejar errores de ingreso de parametros
  try:
    uavb = uav_behaviour(sys.argv[1])
    uavb.run()
  except rospy.ROSInterruptException:
    pass