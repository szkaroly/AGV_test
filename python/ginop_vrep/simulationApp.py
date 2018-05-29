import matplotlib.pyplot as plt
import time
import logging
import numpy as np
from math import cos, sin, atan2, tan


from robot import UnicycleRobot
from ginop_control import DiffDriveKinematics, DiffDriveTrajectoryCommand, TrackingController, generateReferenceInput, generateBezier, DataRecorder
from ginop_control import DataRecorder

class SimulationApp:
    def __init__(self, time = 20):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger('APP')
        self.logger.setLevel('INFO')
        self.logger.debug("Application has started!")
        self.dr = DataRecorder()
        self.sim = DataRecorder()
        self.myRobot = UnicycleRobot(0.15 , 0.45)
        # Create reference trajectory & input velocity
        initial_pos = np.array([[0], [0]])
        initial_tangent = np.array([[5], [0]])
        final_position = np.array([[5], [5]])
        final_tangent = np.array([[5], [2.5]])
        self.dt = 0.05
        self.time = time
        self.reference_trajectory = generateBezier(initial_pos, initial_tangent, final_tangent, final_position, self.dt, self.time)
        self.reference_input = generateReferenceInput(self.reference_trajectory, self.dt)
        InitialPosition = np.array([[0], [0]])
        InitialHeading = initial_tangent
        InitialOrientation = atan2(InitialHeading[1]-InitialPosition[1], InitialHeading[0]-InitialPosition[0])
        self.OldX = np.vstack([InitialPosition, InitialOrientation])

    def getNextCommand(self, index):
        RefVelocity = self.reference_input[0, index]
        RefAngularVelocity = self.reference_input[1, index]
        v, angularVel = self.myRobot.TrackingController.calculateTrackingControl(self.OldX, RefVelocity, RefAngularVelocity, self.reference_trajectory[:, index])
        return v, angularVel


    #Exist the application, closing down the communication
    def exitApp(self):
        self.myRobot.shutdown()
        plt.figure(1)
        plt.subplot(311)
        plt.plot(self.dr.pos['x'], self.dr.pos['y'])
        plt.axis('equal')
        plt.grid(which='major')
        plt.subplot(312)
        v, = plt.plot(self.dr.vel['v'], 'r', label = 'V')
        v_sim, = plt.plot(self.sim.vel['v'], 'black', label = 'V_SIM')
        plt.legend(handles=[v, v_sim])
        plt.grid(which='major')
        plt.subplot(313)
        w, = plt.plot(self.dr.vel['w'], 'b', label = 'W')
        w_sim, = plt.plot(self.sim.vel['w'], 'g', label = 'W_SIM')
        plt.legend(handles=[w, w_sim])
        plt.grid(which='major')
        plt.show()

    def start(self):
        self.logger.info("Start has been called! Starting simulation...")

        try:
            for ii in range((int)(self.time/self.dt)):
                print(ii/(int)(self.time/self.dt) * 100, '%')
                vCmd, wheelAngleCmd = self.getNextCommand(ii)
                if vCmd > 1:
                    vCmd = 1

                tvCmd, twheelAng = self.myRobot.kinematics.InputTransformation(vCmd, wheelAngleCmd)

                self.myRobot.executeControl(vCmd, wheelAngleCmd)
                self.myRobot.executeTrajectory()

                # Pull new values
                v_frontWheel = self.myRobot.frontMotor.getJointVelocity()
                wheel_angle_sim = self.myRobot.steeringMotor.getJointPosition()

                # Calculate new position based on the velocities
                result = self.myRobot.kinematics.integratePosition(self.OldX, self.dt, tvCmd, wheelAngleCmd)
                sim = self.myRobot.kinematics.integratePosition(self.OldX, self.dt, v_frontWheel, wheel_angle_sim)
                self.OldX[0] = result.item(0)
                self.OldX[1] = result.item(1)
                self.OldX[2] = result.item(2)

                self.dr.recordPosition(result.item(0), result.item(1), result.item(2))
                self.sim.recordPosition(sim.item(0), sim.item(1), sim.item(2))
                self.dr.recordVelocity(vCmd, twheelAng, 0, 0 )
                self.sim.recordVelocity(v_frontWheel, wheel_angle_sim, 0 , 0)

        except KeyboardInterrupt:
            self.logger("KeyboardInterrupt received!")
            self.exitApp()

        except Exception as e:
            self.logger(e.with_traceback)
            self.exitApp()
        finally:
            pass

        self.exitApp()


app = SimulationApp()
app.start()
