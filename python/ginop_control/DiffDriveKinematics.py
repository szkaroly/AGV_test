from math import cos, sin, tan
import numpy as np

class DiffDriveKinematics():
    def __init__(self, wheelRadius = 0.45 , l = 1, axisDistance = 0.25):
        self.wheelRadius = wheelRadius # wheel radius of the driven wheels
        self.l = l # distance between fron & rear wheels
        self.axisDistance = axisDistance # distancbe between the two driven wheels
        self.name = 'Diff Drive Kinematics'

    def transformVelocityToWheel(self, linearVel, angularVel):
        vl = (linearVel - angularVel * self.wheelRadius) / self.axisDistance #u1
        vr = (linearVel + angularVel * self.wheelRadius) / self.axisDistance #u2
        return [vr, vl]

    def transformWheelVelocityToRobot(self, vl, vr):
        v = (self.wheelRadius / 2) * (vr+vl)
        omega = (self.wheelRadius/self.axisDistance) * (vr-vl)
        return v, omega


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
        '''
        Calculates the position of the robot after given @time
        Using:
        @param oldPos - previous Position
        @param time - time passed
        @param linVel - linear velocity for that given elapsedTime
        @param wheelAngle - angle of the steering wheel for that given time
        '''
        posDiff = self.calculateDistanceTraveled(time, oldPos, linVel, wheelAngle)
        result = (oldPos+posDiff)
        return result

    def calculateOdometry(self, prevPos, elapsedTime, linearVelocity, angularVelocity):
        x = prevPos[0]
        y = prevPos[1]
        theta = prevPos[2]
        x = x + (linearVelocity * elapsedTime * sin(theta) )
        y = y + (linearVelocity * elapsedTime * cos(theta) )
        theta = theta + ( angularVelocity * elapsedTime )
        return [np.asscalar(x),np.asscalar(y),np.asscalar(theta)]


class DiffDriveTrajectoryCommand:
    def __init__(self, linearVelocity, angularVelocity, steeringAngle, vr, vl):
        self.linearVelocity = np.asscalar(linearVelocity)
        self.angularVelocity = np.asscalar(angularVelocity)
        self.steeringAngle = steeringAngle
        self.vr = np.asscalar(vr)
        self.vl = np.asscalar(vl)

    def printTypes(self):
        print('linearVelocity: {0}, type: {1}'.format(self.linearVelocity,type(self.linearVelocity)))
        print('angularVelocity: {0}, type: {1}'.format(self.angularVelocity,type(self.angularVelocity)))
        print('steeringAngle: {0}, type: {1}'.format(self.steeringAngle,type(self.steeringAngle)))
        print('vr: {0}, type: {1}'.format(self.vr,type(self.vr)))
        print('vl: {0}, type: {1}'.format(self.vl,type(self.vl)))
