

class AllianceBehaviour:

  def __init__(self, n_robots):
    self.motivation = [] # 1 por cada uav (3)
    robots_sets = []

  def calculate_motivation(self):
    for rset in self.motivation:
      impatience = 0
      sensory_feedback = 0
      activity_suppresion = 0
      impatience_reset = 0
      acquiescence = 0

