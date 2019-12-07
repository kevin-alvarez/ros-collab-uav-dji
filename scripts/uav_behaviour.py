#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, UInt8
import sys

class UavBehaviour:

  def __init__(self, uav_id):
    self.loop_rate = 10 # Message frequency (in hz)
    rospy.init_node('uav_behaviour'+uav_id, anonymous=False)

    # Init variables
    self.uav_id = uav_id
    self.neighbor_uav_id = (int(uav_id) % 3) + 1 # 3 total uav, can change
    self.context_data = []
    self.mission_waypoints = []
    self.actual_waypoint = 0

    # Suscriber definition
    rospy.Subscriber('missioncontext', String, self.recv_mission_context)
    rospy.Subscriber('DJIuav{}/missionwaypoints'.format(self.neighbor_uav_id), String, self.recv_mission_waypoints)
    rospy.Subscriber('DJIuav{}/actualwaypoint'.format(self.neighbor_uav_id), UInt8, self.recv_actual_waypoint)
    rospy.Subscriber('DJIuav{}/statusflag'.format(self.neighbor_uav_id), UInt8, self.recv_actual_waypoint)

    # Publisher definition
    self.pub_mission_waypoints = rospy.Publisher('DJIuav'+uav_id+'/newmission', String, queue_size=10)
    self.pub_status_flag = rospy.Publisher('DJIuav'+uav_id+'/statusflag', UInt8, queue_size=10)

    rospy.loginfo("Node uav_behaviour "+uav_id+" ready...")

  def recv_mission_context(self, data):
    self.context_data = data.data.split(";")

  def recv_mission_waypoints(self, data):
    self.mission_waypoints = data.data

  def recv_actual_waypoint(self, data):
    self.actual_waypoint = int(data.data)

  def __define_mission(self):
    # Se debe definir los nuevos (o no) waypoints para la mission

    return "0.100,0.200;0.110,0.200;0.120,0.200"

  def run(self):
    rate = rospy.Rate(self.loop_rate)
    while not rospy.is_shutdown():
      self.pub_mission_waypoints.publish(self.__define_mission())
      rate.sleep()

if __name__ == '__main__': 
  # Falta manejar errores de ingreso de parametros
  try:
    uavb = UavBehaviour(sys.argv[1])
    uavb.run()
  except rospy.ROSInterruptException:
    pass