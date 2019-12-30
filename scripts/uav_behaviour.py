#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, UInt8
import sys
from alliance_behaviour import AllianceBehaviour

class UavBehaviour:

  def __init__(self, uav_id):
    self.loop_rate = 10 # Message frequency (in hz)
    rospy.init_node('uav_behaviour'+uav_id, anonymous=False)

    # Init variables
    self.uav_id = uav_id
    self.neighbor_uav_id = (int(uav_id) % 3) + 1 # 3 total uav, can change
    self.own_data = [0, 0, 0, 0, 0]
    self.neighbor_data = [0, 0, 0, 0, 0]
    self.mission_waypoints = []
    self.mission_stack = []
    self.actual_waypoint = 0
    self.behaviour = AllianceBehaviour()

    # Suscriber definition
    rospy.Subscriber('missioncontext/data{}'.format(self.uav_id), String, self.recv_own_data)
    rospy.Subscriber('missioncontext/data{}'.format(self.neighbor_uav_id), String, self.recv_neighbor_data)
    rospy.Subscriber('DJIuav{}/missionwaypoints'.format(self.neighbor_uav_id), String, self.recv_mission_waypoints)
    rospy.Subscriber('DJIuav{}/actualwaypoint'.format(self.neighbor_uav_id), UInt8, self.recv_actual_waypoint)
    rospy.Subscriber('DJIuav{}/statusflag'.format(self.neighbor_uav_id), UInt8, self.recv_actual_waypoint)
    rospy.Subscriber('DJIuav{}/newmission'.format(self.uav_id), String, self.recv_mission_stack)

    # Publisher definition
    self.pub_mission_waypoints = rospy.Publisher('DJIuav'+uav_id+'/newmission', String, queue_size=10)
    self.pub_status_flag = rospy.Publisher('DJIuav'+uav_id+'/statusflag', UInt8, queue_size=10)

    rospy.loginfo("Node uav_behaviour "+uav_id+" ready...")

  def recv_own_data(self, data):
    self.own_data = data.data.split(",")

  def recv_mission_stack(self, data):
    if data.data != "":
      self.mission_stack = data.data.split(';')
    else:
      self.mission_stack = []

  def recv_neighbor_data(self, data):
    self.neighbor_data = data.data.split(",")

  def recv_mission_waypoints(self, data):
    if data.data != "":
      self.mission_waypoints = data.data.split(';')
    else:
      self.mission_waypoints = []

  def recv_actual_waypoint(self, data):
    self.actual_waypoint = int(data.data)

  def __define_mission(self):
    # Aún no se usan los activadores de comportamientos, solo se toma misión si existe
    self.behaviour.calculate_impatience(self.neighbor_data)
    take_ok = self.behaviour.make_decision()
    if take_ok:
      if not all(wp in self.mission_stack for wp in self.mission_waypoints[self.actual_waypoint:]):
        self.mission_stack.extend(self.mission_waypoints[self.actual_waypoint:])
      new_task = self.mission_stack
    else:
      new_task = self.mission_stack
    new_mission = ";".join(new_task)
    return new_mission

  def run(self):
    rate = rospy.Rate(self.loop_rate)
    while not rospy.is_shutdown():
      rospy.loginfo("data:{} - wp:{} - mission:{}".format(self.own_data, str(self.actual_waypoint), self.mission_waypoints))
      rospy.loginfo("Mission Stack: {}".format(self.mission_stack))
      rospy.loginfo("Impaciencia: {}".format(self.behaviour.impatience))
      self.pub_mission_waypoints.publish(self.__define_mission())
      rate.sleep()

if __name__ == '__main__': 
  # Falta manejar errores de ingreso de parametros
  try:
    uavb = UavBehaviour(sys.argv[1])
    uavb.run()
  except rospy.ROSInterruptException:
    pass
  