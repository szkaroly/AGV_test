from vrepAPIWrapper import vrepCommunicationAPI
from robot import robot
from ginop_control import DiffDriveKinematics, DiffDriveTrajectoryCommand, TrackingController, generateReferenceInput, generateBezier

import time
import logging

import math
from math import cos, sin, atan2, tan
from ginop_control import DataRecorder
import numpy as np


class SimulationApp:
    def __init__(self, time = 10):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger('APP')
        self.logger.setLevel('INFO')
        self.logger.debug("Application has started!")

        self.myRobot = robot()
        self.tc = TrackingController(maxVel=1.25, kinematics=DiffDriveKinematics(d=0.230, l=0.25, axisDistance=0.25))
        self.dr = DataRecorder(tag='norm')
        self.dr_sim = DataRecorder(tag='sim')
        # Create reference trajectory & input velocity
        initial_pos = np.array([[0], [0]])
        initial_tangent = np.array([[10], [0]])
        final_position = np.array([[10], [12]])
        final_tangent = np.array([[10], [10]])
        self.dt = 0.01
        self.time = time
        self.reference_trajectory = generateBezier(initial_pos, initial_tangent, final_tangent, final_position, self.dt, self.time)
        self.reference_input = generateReferenceInput(self.reference_trajectory, self.dt)
        InitialPosition = np.array([[0], [0.1]])
        InitialHeading = initial_tangent
        InitialOrientation = atan2(
            InitialHeading[1]-InitialPosition[1], InitialHeading[0]-InitialPosition[0])
        self.OldX = np.vstack([InitialPosition, InitialOrientation])

    def getNextCommand(self, index):
        RefVelocity = self.reference_input[0, index]
        RefAngularVelocity = self.reference_input[1, index]
        command = self.tc.calculateTrackingControl(self.OldX, RefVelocity, RefAngularVelocity, self.reference_trajectory[:, index])
        return command

    def exitApp(self):
        self.myRobot.val.closeConnection()
        self.dr_sim.save()
        self.dr.save()

    def start(self):
        self.logger.info("Start has been called! Starting simulation...")
        try:
            for ii in range((int)(self.time/self.dt)):
                print('{0}/{1}'.format(ii, (int)(self.time/self.dt)))
                # Fetch next command set
                command = self.getNextCommand(ii)

                # Push commands to VREP & iterate
                self.myRobot.val.setSteeringAngleTarget(command.steeringAngle)
                self.myRobot.appendNewVelocityTrajectory((command.vr, command.vl))
                self.myRobot.executeTrajectory()

                # Pull new values
                vl_sim, vr_sim = self.myRobot.val.getMotorVelocities()
                wheel_angle_sim = self.myRobot.val.getSteeringAngle()

                # Calculate Robot Linear & Angular velocities from wheel
                [velocity_sim, angular_velocity_sim] = self.tc.kinematics.transformWheelVelocityToRobot(vr_sim, vl_sim)

                # Calculate new position based on the velocities
                newPos_reference = self.tc.kinematics.integratePosition(self.OldX, self.dt, command.linearVelocity, command.steeringAngle)
                newPos_sim = self.tc.kinematics.integratePosition(self.OldX, self.dt, velocity_sim, wheel_angle_sim).flatten().tolist()
                newOdo_sim = self.tc.kinematics.calculateOdometry(self.OldX, self.dt, velocity_sim, angular_velocity_sim)

                # Save data
                self.OldX[0] = newPos_sim[0]
                self.OldX[1] = newPos_sim[1]
                self.OldX[2] = newPos_sim[2]

                self.dr_sim.recordSimData(command.linearVelocity, command.angularVelocity, command.steeringAngle,
                                     velocity_sim, angular_velocity_sim, wheel_angle_sim,
                                     vr_sim, vl_sim, command.vr, command.vl)
                self.dr_sim.recordPosition(self.OldX[0].item(0), self.OldX[1].item(0), self.OldX[2].item(0))

        except KeyboardInterrupt:
            self.logger("KeyboardInterrupt received!")
            self.exitApp()

        except Exception as e:
            self.logger(e)
        finally:
            pass

        self.exitApp()


app = SimulationApp()
app.start()
