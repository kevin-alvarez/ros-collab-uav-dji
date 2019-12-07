#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sys

class MissionDataCompiler:

  def __init__(self):
    self.loop_rate = 10 # Message frequency (in hz)
    rospy.init_node('mission_data_compiler', anonymous=False)

    # Init variables
    self.mission_data_1 = ""
    self.mission_data_2 = ""
    self.mission_data_3 = ""
 
    # Suscribers definition
    rospy.Subscriber('missiondata1', String, self.recv_mission_data_1)
    rospy.Subscriber('missiondata2', String, self.recv_mission_data_2)
    rospy.Subscriber('missiondata3', String, self.recv_mission_data_3)

    # Publisher definition
    self.pub_mission_context = rospy.Publisher('missioncontext', String, queue_size=10)

    rospy.loginfo("Node mission compiler ready...")

  def recv_mission_data_1(self, data):
    self.mission_data_1 = data.data

  def recv_mission_data_2(self, data):
    self.mission_data_2 = data.data

  def recv_mission_data_3(self, data):
    self.mission_data_3 = data.data

  def run(self):
    rate = rospy.Rate(self.loop_rate)
    while not rospy.is_shutdown():
      self.pub_mission_context.publish("{};{};{}".format(self.mission_data_1, self.mission_data_2, self.mission_data_3))
      rospy.loginfo("Published Mission Context Info:\n UAV 1: {} \nUAV 2 :{} \nUAV 3: {}".format(self.mission_data_1, self.mission_data_2, self.mission_data_3))
      rate.sleep()

if __name__ == '__main__': 
  # Falta manejar errores de ingreso de parametros
  try:
    mdc = MissionDataCompiler()
    mdc.run()
  except rospy.ROSInterruptException:
    pass
