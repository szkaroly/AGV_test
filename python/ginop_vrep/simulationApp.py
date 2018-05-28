from vrepAPIWrapper import vrepCommunicationAPI
from robot import UnicycleRobot
from ginop_control import DiffDriveKinematics, DiffDriveTrajectoryCommand, TrackingController, generateReferenceInput, generateBezier

import time
import logging

import math
from math import cos, sin, atan2, tan
from ginop_control import DataRecorder
import numpy as np

import matplotlib.pyplot as plt


class SimulationApp:
    def __init__(self, time = 20):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger('APP')
        self.logger.setLevel('INFO')
        self.logger.debug("Application has started!")

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

    def exitApp(self):
        self.myRobot.shutdown()

    def start(self):
        self.logger.info("Start has been called! Starting simulation...")
        self.x = []
        self.y = []
        try:
            for ii in range((int)(self.time/self.dt)):
                print(ii)
                vCmd, wheelAngleCmd = self.getNextCommand(ii)
                if vCmd > 1:
                    vCmd = 1

                self.myRobot.executeControl(vCmd, wheelAngleCmd)
                self.myRobot.executeTrajectory()

                # Pull new values
                v_frontWheel = self.myRobot.frontMotor.getJointVelocity()
                wheel_angle_sim = self.myRobot.steeringMotor.getJointPosition()


                v_frontWheel = v_frontWheel * 0.15
                # Calculate new position based on the velocities
                newPos_reference = self.myRobot.kinematics.integratePosition(self.OldX, self.dt, v_frontWheel, wheelAngleCmd).flatten().tolist()
                self.OldX[0] = newPos_reference[0]
                self.OldX[1] = newPos_reference[1]
                self.OldX[2] = newPos_reference[2]

                self.x.append(self.OldX[0].item(0))
                self.y.append(self.OldX[1].item(0))

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

import pandas as pd
from bokeh.io import show, output_notebook, push_notebook, output_file
from bokeh.plotting import figure
from bokeh.layouts import column

figw = 1250

fig1 = figure(width = figw)
fig1.line(app.x, app.y, legend = 'pos')
#show(fig1)
'''

df = pd.read_csv('simvel.csv')
df.columns

fig1 = figure(width = figw)
fig1.line(df.index, df.commandedLinearVelocity, legend = 'commandedLinearVelocity')
fig1.line(df.index, df.linearVelocitySim, color = 'red', legend = 'linearVelocitySim')
#fig1.line(df.index, df.commandedAngularVelocity, color = 'green', legend = 'commandedAngularVelocity')
#fig1.line(df.index, df.angularVelocitySim, color = 'yellow', legend = 'angularVelocitySim')
fig1.legend.click_policy = 'hide'
'''

'''fig2 = figure(width = figw)
fig2.line(df.index, df.vl_c, legend = 'vlCmd')
fig2.line(df.index, df.vl, legend = 'vlMsr', color = 'red')
fig2.line(df.index, df.vr_c, legend = 'vrCmd', color = 'green')
fig2.line(df.index, df.vr, legend = 'vrMsr', color = 'yellow')
fig2.legend.click_policy = 'hide'

output_file('vel.html')
show(fig1)

figSteer = figure()
figSteer.line(df.index, df.commandedWheelAngle, color = 'blue' , legend = 'wheelAngCmd')
figSteer.line(df.index, df.wheelAngleSim, color = 'red' , legend = 'wheelAngleSim')

df_pos = pd.read_csv('simpos.csv')

figp = figure()
figp.line(df_pos.x, df_pos.y, legend = 'sim')
figp.legend.click_policy = 'hide'
output_file('pos_steer.html')
show(column(figp,figSteer))
'''
