from vrepAPIWrapper import vrepCommunicationAPI
from BezierUtils import generateBezier, generateReferenceInput
from TrackingController import TrackingController
from diffDriveKinematics import DiffDriveKinematics

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
        #self.logger.debug("Sending velocities: %s | %s", vr, vl)
        self.val.triggerStep()


if __name__ == "__main__":
    import math
    from math import cos, sin, atan2, tan
    from dataRercorder import dataRecorder
    import numpy as np
    try:
        myRobot = robot()
        tc = TrackingController()
        dr = dataRecorder()
        dr_sim = dataRecorder()
        #Create reference trajectory & input velocity
        p1 = np.array([[0], [0]])
        p2 = np.array([[1], [0]])
        p3 = np.array([[10], [10]])
        p4 = np.array([[10], [12]])
        dt = 0.01
        time = 10
        reference_trajectory = generateBezier(p1, p2, p3, p4, dt, time)
        reference_input = generateReferenceInput(reference_trajectory, dt)
        InitialPosition = np.array([[0],[0.1]])
        InitialHeading = p2
        InitialOrientation = atan2(InitialHeading[1]-InitialPosition[1],InitialHeading[0]-InitialPosition[0])
        OldX = np.vstack([InitialPosition, InitialOrientation])
        for ii in range((int) (time/dt)):
            RefVelocity = reference_input[0,ii]
            RefAngularVelocity = reference_input[1,ii]
            velocity, wheel_angle, vr, vl = tc.calculateTrackingControl(OldX, RefVelocity, RefAngularVelocity, reference_trajectory[:,ii])
            result = tc.kinematics.integratePosition(OldX, dt, velocity, wheel_angle)
            #TODO Hook it back with received velocities from sim

            #Store new values for next cycle
            OldX[0] = result.item(0)
            OldX[1] = result.item(1)
            OldX[2] = result.item(2)
#            dr.recordPosition(result.item(0), result.item(1), result.item(2))

            myRobot.val.setSteeringAngleTarget(wheel_angle)
            myRobot.appendNewVelocityTrajectory((velocity, velocity))
            myRobot.executeTrajectory()

            vl, vr = myRobot.val.getMotorVelocities()
            ang = myRobot.val.getSteeringAngle()
            dr_sim.recordSimData(velocity, wheel_angle, ang, vr, vl)


    except KeyboardInterrupt:
        print("KeyboardInterrupt received!")
        myRobot.val.closeConnection()
        dr_sim.save()

    except Exception as e:
        print(e)
    finally:
        pass
    myRobot.val.closeConnection()
    dr_sim.save()
