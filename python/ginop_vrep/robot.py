from vrepAPIWrapper import vrepCommunicationAPI
from ginop_control import DiffDriveKinematics, DiffDriveTrajectoryCommand, TrackingController, generateReferenceInput, generateBezier

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
        tc = TrackingController(maxVel = 5, kinematics = DiffDriveKinematics(d = 0.115 , l = 1.25, axisDistance = 0.25))
        dr = DataRecorder(tag = 'norm')
        dr_sim = DataRecorder(tag = 'sim')
        #Create reference trajectory & input velocity
        initial_pos = np.array([[0], [0]])
        initial_tangent = np.array([[10], [0]])
        final_position = np.array([[10], [12]])
        final_tangent = np.array([[10], [10]])
        dt = 0.01
        time = 10
        reference_trajectory = generateBezier(initial_pos, initial_tangent, final_tangent, final_position, dt, time)
        reference_input = generateReferenceInput(reference_trajectory, dt)
        InitialPosition = np.array([[0],[0.1]])
        InitialHeading = initial_tangent
        InitialOrientation = atan2(InitialHeading[1]-InitialPosition[1],InitialHeading[0]-InitialPosition[0])
        OldX = np.vstack([InitialPosition, InitialOrientation])
        for ii in range((int) (time/dt)):
            print('{0}/{1}'.format(ii, (int) (time/dt)))
            RefVelocity = reference_input[0,ii]
            RefAngularVelocity = reference_input[1,ii]


            command = tc.calculateTrackingControl(OldX, RefVelocity, RefAngularVelocity, reference_trajectory[:,ii])

            myRobot.val.setSteeringAngleTarget(command.steeringAngle)
            myRobot.appendNewVelocityTrajectory((command.vr, command.vl))
            myRobot.executeTrajectory()

            vl, vr = myRobot.val.getMotorVelocities()
            sim_wheel_angle = myRobot.val.getSteeringAngle()
            [velocity_sim, angular_velocity_sim] = tc.kinematics.transformWheelVelocityToRobot(vr,vl)
            newPos = tc.kinematics.integratePosition(OldX, dt, command.linearVelocity, command.steeringAngle)
            newOdo = tc.kinematics.calculateOdometry(OldX, dt, velocity_sim, angular_velocity_sim)
            OldX[0] = newPos[0]
            OldX[1] = newPos[1]
            OldX[2] = newPos[2]
#            dr_sim.recordPosition(newPos.item(0), newPos.item(1), newPos.item(2))
            dr_sim.recordSimData(command.linearVelocity, command.angularVelocity, command.steeringAngle,
                                velocity_sim, angular_velocity_sim, sim_wheel_angle,
                                vr, vl, command.vr, command.vl)
            dr_sim.recordPosition(newOdo[0], newOdo[1], newOdo[2])


    except KeyboardInterrupt:
        print("KeyboardInterrupt received!")
        myRobot.val.closeConnection()
        dr_sim.save()
        dr.save()

    except Exception as e:
        print(e)
    finally:
        pass

    myRobot.val.closeConnection()
    dr_sim.save()
