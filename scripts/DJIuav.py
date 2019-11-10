#!/usr/bin/env python

import rospy
from std_msgs.msg import String, UInt8
import sys
import flask

class DJIuav:

  def __init__(self, uav_id):
    self.loop_rate = 10 # Message frequency (in hz)
    rospy.init_node('DJIuav'+uav_id, anonymous=False)

    # Init variables
    self.uav_id = uav_id
    self.mission_waypoints = ""

    # Suscriber definition
    rospy.Subscriber('DJIuav'+uav_id+'/missionwaypoints', String, self.recv_mission_waypoints)

    # Publishers definitions
    self.pub_mission_progress = rospy.Publisher('DJIuav'+uav_id+'/missionprogress', UInt8, queue_size=10)
    self.pub_camera_error = rospy.Publisher('DJIuav'+uav_id+'/cameraerror', UInt8, queue_size=10)
    self.pub_motor_error = rospy.Publisher('DJIuav'+uav_id+'/motorerror', UInt8, queue_size=10)
    self.pub_battery_level = rospy.Publisher('DJIuav'+uav_id+'/batterylevel', UInt8, queue_size=10)

    rospy.loginfo("Node DJIuav "+uav_id+" ready...")

  def recv_mission_waypoints(self, data):
    # transformar string de waypoints a lista...
    self.mission_waypoints = data.data


  def run(self):
    # status variables definition
    mission_progress = 50
    camera_error = 0
    motor_error = 0
    battery_level = 30
    # ROS loop def
    rate = rospy.Rate(self.loop_rate)
    while not rospy.is_shutdown():
      self.pub_mission_progress.publish(mission_progress)
      self.pub_camera_error.publish(camera_error)
      self.pub_motor_error.publish(motor_error)
      self.pub_battery_level.publish(battery_level)
      rospy.loginfo("Mission waypoints: [{}]".format(self.mission_waypoints))
      rate.sleep()

if __name__ == '__main__':
  try:
    djiuav = DJIuav(sys.argv[1])
    djiuav.run()
  except rospy.ROSInterruptException:
    pass
