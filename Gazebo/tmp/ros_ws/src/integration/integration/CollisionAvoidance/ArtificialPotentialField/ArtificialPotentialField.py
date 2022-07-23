import numpy as np

class ArtificialPotentialField:
  def __init__(self, AttGain, RepGain, ForceToPosGain, DistLim):
    self.AttGain = AttGain
    self.RepGain = RepGain
    self.DistLim = DistLim

    self.ForceToPosGain = ForceToPosGain
  def CalAttEnergy(self, Target, Current):
    Norm = np.linalg.norm(Target - Current)

    return Norm

  def CalAttForce(self, Target, Current):
    AttForce = self.AttGain * (Target - Current)

    return AttForce

  def CalRepEnergy(self, Obs, Current):
    ObsDist = np.linalg.norm(Obs - Current)
    RepEnergy = 0
    if ObsDist <= self.DistLim:
      RepEnergy = 0.5 * self.RepGain * (1 / ObsDist - 1 / self.DistLim) ** 2
    else:
      RepEnergy = 0

    return RepEnergy

  def CalRepForce(self, Obs, Current):
    RepForce = 0

    ObsDist = np.linalg.norm(Obs - Current)

    if ObsDist <= self.DistLim:
      RepForce = self.RepGain * (1 / ObsDist - 1 / self.DistLim) * (1 / ObsDist ** 2)
    else:
      RepForce = 0

    return RepForce

  def CalTotalForce(self, Target, Current, Obs):

    Target = np.array(Target)
    Current = np.array(Current)
    Obs = np.array(Obs)

    AttForce = self.CalAttForce(Target, Current)
    RepForce = self.CalRepForce(Obs, Current)

    TotalForce = AttForce + RepForce

    return TotalForce

