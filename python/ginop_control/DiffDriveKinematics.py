from math import cos, sin, tan
import numpy as np

class DiffDriveKinematics():
    def __init__(self, d = 0.45 , l = 2, axisDistance = 0.25):
        self.d = d
        self.l = l
        self.axisDistance = axisDistance
        self.name = 'Diff Drive Kinematics'
        self.jacobian = np.matrix([[self.d/2, self.d/2], [0, 0], [ self.d/self.axisDistance, -self.d/self.axisDistance]]) #2x3

    def transformVelocityToWheel(self, linearVel, angularVel):
        vl = (linearVel - angularVel * self.d) / self.l
        vr = (linearVel + angularVel * self.d) / self.l
        return [vr, vl]

    def transformWheelVelocityToRobot(self, vl, vr):
        q = np.matrix([[vr], [vl]])#  % 2x1
        vel_W = self.jacobian*q    #  % 3x1
        vx = vel_W[0]
        vy = vel_W[1] # This must be always zero for diff drive
        omega = vel_W[2]
        return [vx, omega]


    def calculateDistanceTraveled(self, dt, Position, LinearVelocity, wheelAngle):
        x = Position[0]
        y = Position[1]
        theta = Position[2]

        dx = cos(theta)* LinearVelocity * dt
        dy = sin(theta)* LinearVelocity * dt
        dtheta = tan(wheelAngle) * LinearVelocity * dt / self.l
        result = np.vstack([dx, dy, dtheta])
        return result

    def integratePosition(self, oldPos, time, linVel, wheelAngle):
        posDiff = self.calculateDistanceTraveled(time, oldPos, linVel, wheelAngle)
        result = (oldPos+posDiff)
        return result

    def calculateOdometry(self, prevPos, elapsedTime, linearVelocity, angularVelocity):
        x = prevPos[0]
        x = prevPos[1]
        theta = prevPos[2]
        x = x + (linearVelocity * elapsedTime * sin(theta) )
        y = y + (linearVelocity * elapsedTime * cos(theta) )
        theta = theta + ( angularVelocity * elapsedTime )
        return [x,y,theta]
