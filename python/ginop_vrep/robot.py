import time
import logging

from ginop_vrep import vrepCommunicationAPI, PositionControlledJoint, VelocityControlledJoint, DummyObject
from ginop_control import UnicycleKinematics, DiffDriveTrajectoryCommand, TrackingController, generateReferenceInput, generateBezier


class AbstractRobot(object):
    def __init__(self, TrackingController, HardwareAbstractionLayer, name = 'robot'):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger(name)
        self.logger.setLevel('DEBUG')
        self.logger.debug("Initializing {0}!".format(name))
        self.TrackingController = TrackingController
        self.hal = HardwareAbstractionLayer
        self.hal.initialize()
        self.logger.info("{0} instance has been initialized!".format(name))

    def shutdown(self):
        self.hal.closeConnection()


class UnicycleRobot(AbstractRobot):
    def __init__(self, wheelRadius, L, name = 'UnicycleRobot'):
        self.kinematics = UnicycleKinematics(wheelRadius, L)
        tc = TrackingController(self.kinematics, maxVel = 1, k1 = 0.5, k2 = 0.5, k3 = 1)
        self.frontMotor = VelocityControlledJoint('frontMotor')
        self.steeringMotor = PositionControlledJoint('steeringMotor')
        self.centerPosition = DummyObject('AGV_Center')
        # Putting them into a list , that is passed to the VREP instance
        objects = [self.frontMotor, self.steeringMotor, self.centerPosition]
        #Initialize VREP communication
        HardwareAbstractionLayer = vrepCommunicationAPI(objects)
        AbstractRobot.__init__(self,
                            TrackingController=tc,
                            HardwareAbstractionLayer=HardwareAbstractionLayer,
                            name = name)

    def executeControl(self, vel, angvel):
        """ Maps the given velocity & angular velocity to control inputs, and sets them as a target """
        targetVel, targetSteeringAngle = self.kinematics.InputTransformation(vel, angvel)
        self.frontMotor.setJointVelocity(targetVel)
        self.steeringMotor.setJointPosition(targetSteeringAngle)
        #print('commanding: {0} || {1}'.format(targetVel / self.kinematics.wheelRadius, targetSteeringAngle ))

    def executeTrajectory(self):
        """ Triggers a simulation step in the layer below"""
        self.hal.triggerStep()



class robot(object):
    def __init__(self):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger('robot')
        self.logger.setLevel('DEBUG')
        self.logger.debug("Initializing robot!")
        self.hal = vrepCommunicationAPI()  # Creating VehicleAbstractionLayer
        self.hal.initialize()
        self.logger.info("Robot instance has been initialized!")


if __name__ == "__main__":
    from math import sin, cos
    uniBot = UnicycleRobot(0.2, 1)
    for i in range(0,1000):
        pass
    uniBot.shutdown()
