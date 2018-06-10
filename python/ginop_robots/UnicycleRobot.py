from .AbstractRobot import AbstractRobot

from ginop_vrep import vrepCommunicationAPI, PositionControlledJoint, VelocityControlledJoint, DummyObject
from ginop_control import UnicycleKinematics, TrackingController

class UnicycleRobot(AbstractRobot):
    def __init__(self, wheelRadius, L, name='UnicycleRobot'):
        self.kinematics = UnicycleKinematics(wheelRadius, L)
        tc = TrackingController(self.kinematics, maxVel=1, k1=0.5, k2=0.5, k3=1)
        self.frontMotor = VelocityControlledJoint('frontMotor')
        self.steeringMotor = PositionControlledJoint('steeringMotor')
        self.centerPosition = DummyObject('AGV_Center')
        # Putting them into a list , that is passed to the VREP instance
        objects = [self.frontMotor, self.steeringMotor, self.centerPosition]
        # Initialize VREP communication
        HardwareAbstractionLayer = vrepCommunicationAPI(objects)

        AbstractRobot.__init__(self,
                               TrackingController=tc,
                               HardwareAbstractionLayer=HardwareAbstractionLayer,
                               name=name)

    def executeControl(self, vel, angvel):
        """ Maps the given velocity & angular velocity to control inputs, and sets them as a target """
        targetVel, targetSteeringAngle = self.kinematics.InputTransformation(vel, angvel)
        self.frontMotor.setJointVelocity(targetVel / self.kinematics.wheelRadius)
        self.steeringMotor.setJointPosition(targetSteeringAngle)
        # print('commanding: {0} || {1}'.format(targetVel / self.kinematics.wheelRadius, targetSteeringAngle ))

    def executeTrajectory(self):
        """ Triggers a simulation step in the layer below"""
        self.hal.triggerStep()


if __name__ == "__main__":
    from math import sin, cos

    uniBot = UnicycleRobot(0.2, 1)
    for i in range(0, 1000):
        pass
    uniBot.shutdown()
