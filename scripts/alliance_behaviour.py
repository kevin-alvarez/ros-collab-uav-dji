#!/usr/bin/env python3

class AllianceBehaviour:
  def __init__(self):
    # Data
    self.mission_progress = 0
    self.uav_data = []

    # Variables
    self.impatience = 0
    self.motivation = 0
    self.motivation_limit = 500 # a 10hz se demora 50s

  def calculate_impatience(self, uav_data): #UAV data: id, missionprogress, camera, motors, batterylevel
    self.uav_data = uav_data
    new_progress = uav_data[1]
    if new_progress not in ('0', '100'):
      if new_progress == self.mission_progress: # El progreso no ha cambiado
        self.impatience += 1 # Impaciencia con crecimiento uniforme
      else:
        self.impatience = 0 # Reset de impaciencia
        self.mission_progress = new_progress
    else:
      self.impatience = 0

  def make_decision(self):
    camera = int(self.uav_data[2])
    motors = int(self.uav_data[3])
    if camera == 1 or motors == 1:
      sensory_feedback = 1
    else:
      sensory_feedback = 0
    self.motivation = self.impatience * sensory_feedback # 0 si camera o motors es 0
    if self.motivation >= self.motivation_limit:
      self.impatience = 0
      return True
    return False
