#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, UInt8
import sys

class UavData:

  def __init__(self, uav_id):
    self.loop_rate = 10 # Message frequency (in hz)
    rospy.init_node('uav_data'+uav_id, anonymous=False)

    # Init variables
    self.uav_id = uav_id
    self.mission_progress = 0
    self.camera_status = 0
    self.motor_status = 0
    self.battery_level = 0

    # Suscribers definition
    rospy.Subscriber('DJIuav'+uav_id+'/missionprogress', UInt8, self.recv_mission_progress)
    rospy.Subscriber('DJIuav'+uav_id+'/camerastatus', UInt8, self.recv_camera_status)
    rospy.Subscriber('DJIuav'+uav_id+'/motorstatus', UInt8, self.recv_motor_status)
    rospy.Subscriber('DJIuav'+uav_id+'/batterylevel', UInt8, self.recv_battery_level)

    # Publisher definition
    self.pub_mission_data = rospy.Publisher('missiondata'+uav_id, String, queue_size=10)

    rospy.loginfo("Node uav_data "+uav_id+" ready...")

  def recv_mission_progress(self, data):
    self.mission_progress = data.data

  def recv_camera_status(self, data):
    self.camera_status = data.data

  def recv_motor_status(self, data):
    self.motor_status = data.data

  def recv_battery_level(self, data):
    self.battery_level = data.data

  def __get_mission_data(self):
    mission_data = "{},{},{},{},{}".format(self.uav_id, self.mission_progress, self.camera_status, self.motor_status, self.battery_level)
    return mission_data

  def run(self):
    rate = rospy.Rate(self.loop_rate)
    while not rospy.is_shutdown():
      # Logs
      self.pub_mission_data.publish(self.__get_mission_data())
      rate.sleep()
      

if __name__ == '__main__': 
  # Falta manejar errores de ingreso de parametros
  try:
    uavd = UavData(sys.argv[1])
    uavd.run()
  except rospy.ROSInterruptException:
    pass
