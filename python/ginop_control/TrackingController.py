import numpy as np
import time
import logging
from math import cos, sin, atan2, tan

from .BezierUtils import *
from .Kinematics import DiffDriveKinematics, DiffDriveTrajectoryCommand, UnicycleKinematics, UnicycleTrajectoryCommand


pi = 3.14

class TrackingController():
    def __init__(self, kinematics = DiffDriveKinematics(), maxVel = 1.5, k1 = 0.5, k2 = 0.5, k3 = 1):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger('trackingCtrl')
        self.logger.setLevel('INFO')
        self.logger.debug("Initializing TrackingController with Kinematics: {0}!".format(kinematics.name))

        #Feedback gains
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3

        self.MaxVelocity = np.matrix([maxVel])
        self.kinematics = kinematics

    def calculateTrackingControl(self, OldX, RefVelocity, RefAngularVelocity, ReferenceTrajectory):
        '''
        OldX - [X,Y,Theta] 3x1 matrix containing the previous position
        RefVelocity [v] - 1x1 matrix containing the expected velocity for the given iteration
        RefAngularVelocity [w] - 1x1 matrix containing the expected angular velocity for the given iteration
        ReferenceTrajectory - 3x1 matrix containing the x,y,theta for the given iteration
        '''
        xk = OldX[0] #x,k spatial
        yk = OldX[1] #y,k spatial
        phik = OldX[2] #phi,k spatial

        xr = ReferenceTrajectory[0] #x,ref,k+1 spatial
        yr = ReferenceTrajectory[1] #x,ref,k+1 spatial
        phir = ReferenceTrajectory[2] #phi,ref,k+1 spatial

        ek = np.matrix([xk-xr,yk-yr]) ##tracking error, spatial
        eref = np.linalg.lstsq(planarRot(phir),ek, rcond = None)[0] # Tracking error reference -> Result of Left array division
        erx = eref[0]; #e,x reference
        ery = eref[1]; #e,y reference
        orib = np.mod(np.mod(phik,2*pi)-np.mod(phir,2*pi),2*pi); #orientation error, reference


        #Calculate actual velocities & angles
        velocity = (RefVelocity-self.k1*abs(RefVelocity)*(erx+ery*cos(orib)))/cos(orib)
#        if velocity > self.MaxVelocity: #saturaton on the linear velocity
#            velocity = self.MaxVelocity
        angularVelocity = RefAngularVelocity-(((self.k2*RefVelocity*ery)+(self.k3*abs(RefVelocity)*tan(orib)))*(cos(orib)*cos(orib)));
        wheelAngle = atan2(angularVelocity*self.kinematics.L,velocity);
        return velocity.item(0), wheelAngle

if __name__ == "__main__":
    import unittest
    from DataRecorder import DataRecorder
    class testTrackingController():

        # Simple test case for checking the output of controller
        def testCalculateTrackingControl():
            #Create logger
            mylogger = logging.getLogger('test')
            FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
            logging.basicConfig(format=FORMAT)
            mylogger.setLevel('DEBUG')
            dr = DataRecorder()
            tc = TrackingController()

            #Create reference trajectory & input velocity
            p1 = np.array([[0], [0]])
            p2 = np.array([[10], [0]])
            p3 = np.array([[10], [10]])
            p4 = np.array([[10], [12]])
            dt = 0.01
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
                command = tc.calculateTrackingControl(OldX, RefVelocity, RefAngularVelocity, reference_trajectory[:,ii])
                result = tc.kinematics.integratePosition(OldX, dt, command.linearVelocity, command.steeringAngle)
                #Store new values for next cycle
                OldX[0] = result.item(0)
                OldX[1] = result.item(1)
                OldX[2] = result.item(2)
                dr.recordPosition(result.item(0), result.item(1), result.item(2))
                dr.recordVelocity(command.linearVelocity, command.steeringAngle, command.vr, command.vl)
            dr.save()

    tc = testTrackingController
    tc.testCalculateTrackingControl()
