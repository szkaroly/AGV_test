import numpy as np
from BezierUtils import *
from math import cos, sin, atan2, tan
from dataRercorder import dataRecorder

import time
import logging


pi = 3.14

class DiffDriveKinematics():
    def __init__(self, d = 0.15 , l = 0.1 ):
        self.d = d
        self.l = l
        self.name = 'Diff Drive Kinematics'

    def transformVelocityToWheel(self, linearVel, angularVel):
        vl = (linearVel - angularVel * self.d) / self.l
        vr = (linearVel + angularVel *self.d) / self.l
        return [vr, vl]



    def TargoncaKinematics(self, dt, XVec, LinearVelocity, wheelAngle):
        x = XVec[0]
        y = XVec[1]
        theta = XVec[2]

        dx = cos(theta)* LinearVelocity * dt
        dy = sin(theta)*LinearVelocity * dt
        dtheta = tan(wheelAngle)*LinearVelocity * dt / self.l
        result = np.vstack([dx, dy, dtheta])
        return result

    def integratePosition(self, oldPos, time, linVel, wheelAngle):
        posDiff = self.TargoncaKinematics(time, oldPos, linVel, wheelAngle)
        result = (oldPos+posDiff)
        return result

class TrackingController():
    def __init__(self, kinematics = DiffDriveKinematics()):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger('trackingCtrl')
        self.logger.setLevel('DEBUG')
        self.logger.debug("Initializing TrackingController with Kinematics: {0}!".format(kinematics.name))

        #Feedback gains
        self.k1 = 0.5
        self.k2 = 0.5
        self.k3 = 1
        self.MaxVelocity = np.matrix([1.5])
        self.kinematics = kinematics

    def calculateTrackingControl(self, OldX, RefVelocity, RefAngularVelocity, ReferenceTrajectory):
        '''
        OldX - [X,Y,Theta] 3x1 matrix
        RefVelocity [v] - 1x1 matrix
        RefAngularVelocity [w] - 1x1 matrix
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

        if (self.k1==0): #for testing if the reference velocities are correct #TODO Remove this..
            velocity = RefVelocity;
        else: #the actual feedback control
            velocity = (RefVelocity-self.k1*abs(RefVelocity)*(erx+ery*cos(orib)))/cos(orib)
        if velocity > self.MaxVelocity: #saturaton on the linear velocity
            velocity = self.MaxVelocity

        print(type(velocity))
        angularVelocity = RefAngularVelocity-(((self.k2*RefVelocity*ery)+(self.k3*abs(RefVelocity)*tan(orib)))*(cos(orib)*cos(orib)));
        wheelAngle=atan2(angularVelocity*self.kinematics.l,velocity);
        vr,vl=  self.kinematics.transformVelocityToWheel(velocity, angularVelocity)
        return [velocity.item(0), wheelAngle, vr.item(0), vl.item(0)]

if __name__ == "__main__":
    import unittest
    import matplotlib.pyplot as plt
    class testTrackingController():
        # This checks a simple bezier spline start/end position & orientation
        def testCalculateTrackingControl():
            #Create logger
            mylogger = logging.getLogger('test')
            FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
            logging.basicConfig(format=FORMAT)
            mylogger.setLevel('DEBUG')
            dr = dataRecorder()
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
                velocity, wheel_angle, vr, vl = tc.calculateTrackingControl(OldX, RefVelocity, RefAngularVelocity, reference_trajectory[:,ii])


                result = tc.kinematics.integratePosition(OldX, dt, velocity, wheel_angle)

                #Store new values for next cycle
                OldX[0] = result.item(0)
                OldX[1] = result.item(1)
                OldX[2] = result.item(2)
                dr.recordPosition(result.item(0), result.item(1), result.item(2))
                dr.recordVelocity(velocity,wheel_angle, vr, vl)
            dr.save()

tc = testTrackingController
tc.testCalculateTrackingControl()
