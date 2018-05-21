from math import cos, sin, tan
import numpy as np

class AGVKinematics(object):
    def __init__(self, wheelRadius, L, name = 'unnamed'):
        self.wheelRadius = wheelRadius # wheel radius of the driven wheels
        self.L = L # distance between fron & rear wheels
        self.name = name

    def transformVelocityToWheel(self, linearVel, angularVel):
        '''
        This method calculates the wheel velociries based on the the expected linear/angular velocity of the robot
        '''
        raise NotImplementedError('transformVelocityToWheel function not implemented')

    def transformWheelVelocityToRobot(self, wheelVelocities):
        '''
        This method calculates the velocity of the robot based on the wheel velocities
        @param wheelVelocities : list of wheel velocities in the defined order
        '''
        raise NotImplementedError('transformWheelVelocityToRobot function not implemented!')

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


class DiffDriveKinematics(AGVKinematics):
    def __init__(self, wheelRadius = 0.45 , L = 1, axisDistance = 0.25):
        AGVKinematics.__init__(self, wheelRadius = wheelRadius, L = L, name = 'DiffDriveKinematics')
        self.axisDistance = axisDistance # distancbe between the two driven wheels

    def transformVelocityToWheel(self, linearVel, angularVel):
        vl = (linearVel - angularVel * self.wheelRadius) / self.axisDistance #u1
        vr = (linearVel + angularVel * self.wheelRadius) / self.axisDistance #u2
        return vr, vl

    def transformWheelVelocityToRobot(self, wheelVelocities):
        vr = wheelVelocities[0]
        vl = wheelVelocities[1]
        v = (self.wheelRadius / 2) * (vr+vl)
        omega = (self.wheelRadius/self.axisDistance) * (vr-vl)
        return v, omega


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


class UnicycleKinematics(AGVKinematics):
    def __init__(self, wheelRadius = 0.1, L = 1):
        AGVKinematics.__init__(self, wheelRadius = wheelRadius, L = L, name = 'Unicycle Kinematics')

    def transformVelocityToWheel(self, linearVel , angularVel = 0):
        wheel_angular_velocity = linearVel / self.wheelRadius
        return wheel_angular_velocity

    def transformWheelVelocityToRobot(self, wheelVelocities):
        '''Note:
        Unicycle model does not have angular velocity, only steering angles
        Therefore this model does not provide any value for the angular velocity
        '''
        wheel_angular_velocity = wheelVelocities[0]
        v = wheel_angular_velocity * self.wheelRadius
        return v, 0

class UnicycleTrajectoryCommand:
    def __init__(self, wheelVelocity, steeringAngleTarget):
        self.linearVelocityTarget = wheelVelocity
        self.steeringAngleTarget = steeringAngleTarget

uk = UnicycleKinematics()
