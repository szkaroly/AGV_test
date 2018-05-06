from math import cos, sin, tan
import numpy as np

class DiffDriveKinematics():
    def __init__(self, d = 0.45 , l = 2):
        self.d = d
        self.l = l
        self.name = 'Diff Drive Kinematics'

    def transformVelocityToWheel(self, linearVel, angularVel):
        vl = (linearVel - angularVel * self.d) / self.l
        vr = (linearVel + angularVel * self.d) / self.l
        return [vr, vl]

    def transformWheelVelocityToRobot(self, vl, vr):
        pass

    def calculateDistanceTraveled(self, dt, Position, LinearVelocity, wheelAngle):
        x = Position[0]
        y = Position[1]
        theta = Position[2]

        dx = cos(theta)* LinearVelocity * dt
        dy = sin(theta)*LinearVelocity * dt
        dtheta = tan(wheelAngle)*LinearVelocity * dt / self.l
        result = np.vstack([dx, dy, dtheta])
        return result

    def integratePosition(self, oldPos, time, linVel, wheelAngle):
        posDiff = self.calculateDistanceTraveled(time, oldPos, linVel, wheelAngle)
        result = (oldPos+posDiff)
        return result
