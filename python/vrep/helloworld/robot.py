from vrepAPIWrapper import vrepCommunicationAPI
from BezierUtils import generateBezier, generateReferenceInput
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

    def executeTrajectory(self):
        vr, vl = self.vc.getNextVelocities()
        self.val.setMotorVelocities(vr, vl)
        self.logger.debug("Sending velocities: %s | %s", vr, vl)
        self.val.triggerStep()


if __name__ == "__main__":
    import math

    try:
        myRobot = robot()
        count = 0
        while True:
            a = math.sin(count * 0.0174532925)
            count = count + 1
            myRobot.appendNewVelocityTrajectory((a, a * 1.2))
            myRobot.executeTrajectory()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received!")
        myRobot.val.closeConnection()
    except Exception as e:
        print(e)
    finally:
        pass

