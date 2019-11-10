#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys

class UavBehaviour:

  def __init__(self, uav_id):
    rospy.init_node('uav_behaviour'+uav_id, anonymous=False)

    # Init variables
    self.mission_context_data = ""

    # Suscriber definition
    rospy.Subscriber('missioncontext', String, self.recv_mission_context)

    # Publisher definition
    self.pub_actual_command = rospy.Publisher('DJIuav'+uav_id+'/missionwaypoints', String, queue_size=10)

    rospy.loginfo("Node uav_behaviour "+uav_id+" ready...")

  def recv_mission_context(self, data):
    self.mission_context_data = data.data

  def __define_mission(self):
    # Se debe definir los nuevos (o no) waypoints para la mission
    return "0.100,0.200;0.110,0.200;0.120,0.200"

  def run(self):
    while not rospy.is_shutdown():
      self.pub_actual_command.publish(self.__define_mission())

if __name__ == '__main__': 
  # Falta manejar errores de ingreso de parametros
  try:
    uavb = UavBehaviour(sys.argv[1])
    uavb.run()
  except rospy.ROSInterruptException:
    pass