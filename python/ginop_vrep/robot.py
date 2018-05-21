from ginop_vrep import vrepCommunicationAPI, PositionControlledJoint, VelocityControlledJoint
from ginop_control import UnicycleKinematics, DiffDriveTrajectoryCommand, TrackingController, generateReferenceInput, generateBezier

from abc import ABCMeta, abstractmethod

import time
import logging


class velocityContainer(object):
    def __init__(self):
        self.vr = []
        self.vl = []

    def getNextVelocities(self) -> tuple:
        """
        :return: next velocities in the container, zero if there is none
        """
        if len(self.vr) == 0:
            return 0, 0
        else:
            return self.vr.pop(), self.vl.pop()

    def appendNewVelocityTrajectory(self, inputTrajectory):
        self.vr.append(inputTrajectory[0])
        self.vl.append(inputTrajectory[1])

class AbstractRobot(object):
    def __init__(self, TrackingController, VehicleAbstractionLayer, name = 'robot'):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger(name)
        self.logger.setLevel('DEBUG')
        self.logger.debug("Initializing {0}!".format(name))
        self.TrackingController = TrackingController
        self.val = VehicleAbstractionLayer
        self.val.initialize()
        self.velocityContainer = velocityContainer()
        self.logger.info("{0} instance has been initialized!".format(name))

    def shutdown(self):
        self.val.closeConnection()


class UnicycleRobot(AbstractRobot):
    def __init__(self, wheelRadius, L, name = 'UnicycleRobot'):
        kinematics = UnicycleKinematics(wheelRadius, L)
        tc = TrackingController(kinematics, maxVel = 1.5, k1 = 0.5, k2 = 0.5, k3 = 1)
        self.frontMotor = VelocityControlledJoint('frontMotor')
        self.steeringMotor = PositionControlledJoint('steeringMotor')
        joints = [self.frontMotor, self.steeringMotor]
        VehicleAbstractionLayer = vrepCommunicationAPI(joints)
        self.vc = velocityContainer()
        AbstractRobot.__init__(self,
                            TrackingController=tc,
                            VehicleAbstractionLayer=VehicleAbstractionLayer,
                            name = name)

    def appendNewVelocityTrajectory(self, trajectory):
        self.vc.appendNewVelocityTrajectory(trajectory)

    def executeTrajectory(self):
        vr, vl = self.vc.getNextVelocities()
        self.frontMotor.setJointVelocity(vr)
        self.steeringMotor.setJointPosition(vl)
        self.val.triggerStep()


class robot(object):
    def __init__(self):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger('robot')
        self.logger.setLevel('DEBUG')
        self.logger.debug("Initializing robot!")
        self.val = vrepCommunicationAPI()  # Creating VehicleAbstractionLayer
        self.val.initialize()
        self.vc = velocityContainer()
        self.logger.info("Robot instance has been initialized!")

    def appendNewVelocityTrajectory(self, trajectory):
        self.vc.appendNewVelocityTrajectory(trajectory)


if __name__ == "__main__":
    from math import sin, cos
    uniBot = UnicycleRobot(0.2, 1)
    for i in range(0,1000):
        uniBot.appendNewVelocityTrajectory((sin(i/10), cos(i/10)))
        uniBot.executeTrajectory()
    uniBot.shutdown()
