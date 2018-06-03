import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import time
import logging
import numpy as np
from math import cos, sin, atan2, tan


from robot import UnicycleRobot
from ginop_control import DiffDriveKinematics, TrackingController, generateReferenceInput, generateBezier, DataRecorder
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
        self.myRobot = UnicycleRobot(0.15/2 , 0.4)
        # Create reference trajectory & input velocity
        initial_pos = np.array([[0], [0]])
        initial_tangent = np.array([[5], [0]])
        final_position = np.array([[5], [5]])
        final_tangent = np.array([[5], [2.5]])
        self.dt = 0.1
        self.time = time
        self.reference_trajectory = generateBezier(initial_pos, initial_tangent, final_tangent, final_position, self.dt, self.time)
        self.reference_input = generateReferenceInput(self.reference_trajectory, self.dt)
        InitialPosition = np.array([[0], [0]])
        InitialHeading = initial_tangent
        InitialOrientation = atan2(InitialHeading[1]-InitialPosition[1], InitialHeading[0]-InitialPosition[0])
        self.OldX = np.vstack([InitialPosition, InitialOrientation])
        self.logger.info("Initialization is finished!")

    def getNextCommand(self, index):
        RefVelocity = self.reference_input[0, index]
        RefAngularVelocity = self.reference_input[1, index]
        v, angularVel = self.myRobot.TrackingController.calculateTrackingControl(self.OldX, RefVelocity, RefAngularVelocity, self.reference_trajectory[:, index])
        return v, angularVel


    def frange(self, start, end=None, inc=None):
        "A range function, that does accept float increments..."

        if end == None:
            end = start + 0.0
            start = 0.0

        if inc == None:
            inc = 1.0

        L = []
        while 1:
            next = start + len(L) * inc
            if inc > 0 and next >= end:
                break
            elif inc < 0 and next <= end:
                break
            L.append(next)

        return L

    #Exist the application, closing down the communication
    def exitApp(self):
        print('\n')
        self.myRobot.shutdown()
        plt.close('all')
        plt.figure(1)
        plt.subplot(311)
        reference_pos, = plt.plot(self.reference_trajectory[0], self.reference_trajectory[1], 'green', label = 'reference position')
        integr_pos, =plt.plot(self.dr.pos['x'], self.dr.pos['y'], label = 'integrated position')
        sim_center_pos, = plt.plot(self.sim.pos['x'], self.sim.pos['y'],'red', label = 'simulation center pos')
        plt.legend(handles=[reference_pos ,integr_pos, sim_center_pos])
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

    def progressBar(self, value, endvalue, bar_length=20):

        percent = float(value) / endvalue
        arrow = '-' * int(round(percent * bar_length)-1) + '>'
        spaces = ' ' * (bar_length - len(arrow))

        sys.stdout.write("\rPercent: [{0}] {1}%".format(arrow + spaces, int(round(percent * 100))))
        sys.stdout.flush()

    def start(self):
        self.logger.info("Start has been called! Starting simulation...")
        try:

            print('\n')
            index = 0
            for time in self.frange(0, self.time + self.dt, self.dt):
                self.progressBar(time, self.time+ self.dt, 30)
                vCmd, wheelAngleCmd = self.getNextCommand(index)

                tvCmd, twheelAng = self.myRobot.kinematics.InputTransformation(vCmd, wheelAngleCmd)

                self.myRobot.executeControl(vCmd, wheelAngleCmd)
                self.myRobot.executeTrajectory()

                # Pull new values
                v_frontWheelJointVelocity = self.myRobot.frontMotor.getJointVelocity()
                wheel_angle_sim = self.myRobot.steeringMotor.getJointPosition()
                v_frontWheel = v_frontWheelJointVelocity * self.myRobot.kinematics.wheelRadius

                # Calculate new position based on the velocities
                result = self.myRobot.kinematics.integratePosition(self.OldX, self.dt, tvCmd, wheelAngleCmd)
                sim    = self.myRobot.kinematics.integratePosition(self.OldX, self.dt, v_frontWheel, wheel_angle_sim)
                centerPosition = self.myRobot.centerPosition.getObjectPosition()
                self.OldX[0] = centerPosition[0]
                self.OldX[1] = centerPosition[1]
                self.OldX[2] = centerPosition[2]

                '''
                self.OldX[0] = result.item(0)
                self.OldX[1] = result.item(1)
                self.OldX[2] = result.item(2)
                '''

                self.dr.recordPosition(result.item(0), result.item(1), result.item(2))
                self.sim.recordPosition(centerPosition[0], centerPosition[1], centerPosition[2])
                self.dr.recordVelocity(vCmd, twheelAng, 0, 0 )
                self.sim.recordVelocity(v_frontWheel, wheel_angle_sim, 0 , 0)

                index = index + 1

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
