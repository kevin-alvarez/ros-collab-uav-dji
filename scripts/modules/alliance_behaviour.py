

class AllianceBehaviour:
  def __init__(self, n_robots):
    # Data
    self.mission_progress = 0

    # Variables
    self.impatience = 0
    self.motivation = 0
    self.motivation_limit = 10000

  def calculate_impatience(self, new_progress):
    if self.mission_progress != 0:
      if new_progress == self.mission_progress: # El progreso no ha cambiado
        self.impatience += 1 # Impaciencia con crecimiento uniforme
      else:
        self.impatience = 0 # Reset de impaciencia

  def calculate_motivation(self, ):
    self.motivation = self.impatience
    if self.motivation >= self.motivation_limit: return 1
    return 0

