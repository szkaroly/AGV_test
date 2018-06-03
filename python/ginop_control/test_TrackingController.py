import unittest
import logging
import matplotlib.pyplot as plt
import numpy as np
from math import atan2
from TrackingController import TrackingController
from BezierUtils import generateBezier, generateReferenceInput
from Kinematics import UnicycleKinematics
from DataRecorder import DataRecorder

class testTrackingController(unittest.TestCase):

    # Simple test case for checking the output of controller
    def generateReferenceVelocityAndPosTrajectory(self):
        ''' This function generates a reference trajectory plot for position & velocity of the AGV '''
        mylogger = logging.getLogger('test')
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        mylogger.setLevel('DEBUG')
        dr = DataRecorder()
        tc = TrackingController(kinematics = UnicycleKinematics(0.15 , 0.45))

        #Create reference trajectory & input velocity
        p1 = np.array([[0], [0]])
        p2 = np.array([[5], [0]])
        p4 = np.array([[5], [5]])
        p3 = np.array([[5], [2.5]])
        dt = 0.05
        time = 20
        reference_trajectory = generateBezier(p1, p2, p3, p4, dt, time)
        reference_input = generateReferenceInput(reference_trajectory, dt)
        InitialPosition = np.array([[0],[0.1]])
        InitialHeading = p2
        InitialOrientation = atan2(InitialHeading[1]-InitialPosition[1],InitialHeading[0]-InitialPosition[0])
        OldX = np.vstack([InitialPosition, InitialOrientation])
        for ii in range((int) (time/dt)):
            RefVelocity = reference_input[0,ii]
            RefAngularVelocity = reference_input[1,ii]
            lin, ang = tc.calculateTrackingControl(OldX, RefVelocity, RefAngularVelocity, reference_trajectory[:,ii])
            result = tc.kinematics.integratePosition(OldX, dt, lin, ang)
            #Store new values for next cycle
            OldX[0] = result.item(0)
            OldX[1] = result.item(1)
            OldX[2] = result.item(2)
            dr.recordPosition(result.item(0), result.item(1), result.item(2))
            dr.recordVelocity(lin, ang, 0, 0 )

        plt.figure(1)
        plt.subplot(211)
        plt.plot(dr.pos['x'], dr.pos['y'])
        plt.axis('equal')
        plt.grid(which='major')
        plt.subplot(212)
        v, = plt.plot(dr.vel['v'], 'r', label = 'V')
        w, = plt.plot(dr.vel['w'], 'b', label = 'W')
        plt.legend(handles=[v, w])
        plt.grid(which='major')
        plt.show()

tc = testTrackingController()
tc.generateReferenceVelocityAndPosTrajectory()
