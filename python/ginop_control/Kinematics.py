from math import cos, sin, tan, atan2
import numpy as np

class AGVKinematics(object):
    def __init__(self, wheelRadius, L, name = 'unnamed'):
        self.wheelRadius = wheelRadius # wheel radius of the driven wheels
        self.L = L # distance between fron & rear wheels
        self.name = name

    def InputTransformation(self, velocity, angularVelocity):
        raise NotImplemented("InputTransformation not implemented!")

    def calculateDistanceTraveled(self, dt, Position, LinearVelocity, wheelAngle):
        x = Position[0]
        y = Position[1]
        theta = Position[2]

        dx = cos(theta)* LinearVelocity * dt
        dy = sin(theta)* LinearVelocity * dt
        dtheta = tan(wheelAngle) * LinearVelocity * dt / self.L
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

'''
% type = 3: differentially driven car, in this case the inputs and outputs are:
%   control1: the angular velocity of the left wheel
%   control2: the angular velocity of the right wheel
'''
class DiffDriveKinematics(AGVKinematics):
    def __init__(self, wheelRadius = 0.45 , L = 1, axisDistance = 0.25):
        AGVKinematics.__init__(self, wheelRadius = wheelRadius, L = L, name = 'DiffDriveKinematics')
        self.axisDistance = axisDistance # distancbe between the two driven wheels

    def InputTransformation(self, velocity, angularVelocity):
        D = self.axisDistance
        r = self.wheelRadius
        control1 = (velocity - angularVelocity * D) /r
        control2 = (velocity + angularVelocity * D) /r
        return control1, control2

'''
% type == 1: steered front wheel, in this case the inputs and outputs are:
%   L: distance between the axes of the front and rear wheels
%   control1: the linear velocity of the car
%   control2: the wheel angle
'''
class UnicycleKinematics(AGVKinematics):
    def __init__(self, wheelRadius = 0.1, L = 1):
        AGVKinematics.__init__(self, wheelRadius = wheelRadius, L = L, name = 'Unicycle Kinematics')

    def InputTransformation(self, velocity, angularVelocity):
        control1 = velocity
        control2 = atan2(angularVelocity*self.L, velocity)
        return control1, control2
