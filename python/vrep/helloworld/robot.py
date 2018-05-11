from vrepAPIWrapper import vrepCommunicationAPI
from ginop_control import DiffDriveKinematics, TrackingController, generateReferenceInput, generateBezier

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
    from ginop_control import DataRecorder
    import numpy as np
    try:
        myRobot = robot()
        tc = TrackingController(maxVel =5)
        dr = dataRecorder()
        dr_sim = dataRecorder()
        #Create reference trajectory & input velocity
        initial_pos = np.array([[0], [0]])
        initial_tangent = np.array([[10], [0]])
        final_position = np.array([[10], [10]])
        final_tangent = np.array([[10], [5]])
        dt = 0.01
        time = 10
        reference_trajectory = generateBezier(initial_pos, initial_tangent, final_tangent, final_position, dt, time)
        reference_input = generateReferenceInput(reference_trajectory, dt)
        InitialPosition = np.array([[0],[0.1]])
        InitialHeading = initial_tangent
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
