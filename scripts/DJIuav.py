#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys

class DJIuav:

  def __init__(self, uav_id):
    self.loop_rate = 10 # Message frequency (in hz)
    rospy.init_node('DJIuav'+uav_id, anonymous=False)

    # Init variables
    self.mission_actual_command = ""

    # Suscriber definition
    rospy.Subscriber('DJIuav'+uav_id+'/actualcommand', String, self.recv_actual_command)

    # Publishers definitions
    self.pub_mission = rospy.Publisher('DJIuav'+uav_id+'/missionprogress', String, queue_size=10)
    self.pub_camera = rospy.Publisher('DJIuav'+uav_id+'/camerastatus', String, queue_size=10)
    self.pub_motor = rospy.Publisher('DJIuav'+uav_id+'/motorstatus', String, queue_size=10)

    rospy.loginfo("Node DJIuav "+uav_id+" ready...")

  def recv_actual_command(self, data):
    self.mission_actual_command = data.data

  def run(self):
    # ROS loop def
    rate = rospy.Rate(self.loop_rate)
    while not rospy.is_shutdown():
      #rospy.loginfo("Loggin info on topics...")
      self.pub_mission.publish("mission_status:OK")
      self.pub_camera.publish("camera_status:OK")
      self.pub_motor.publish("motor_status:OK")
      #rospy.loginfo("All status logged!")
      rospy.loginfo("Actual Command: "+self.mission_actual_command)
      rate.sleep()  

if __name__ == '__main__':
  # Falta manejar errores de ingreso de parametros
  try:
    djiuav = DJIuav(sys.argv[1])
    djiuav.run()
  except rospy.ROSInterruptException:
    pass