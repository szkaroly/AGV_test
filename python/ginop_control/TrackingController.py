import numpy as np
import time
import logging
from math import cos, sin, atan2, tan, pi

from .Utilities import *
from .Kinematics import DiffDriveKinematics, UnicycleKinematics



class TrackingController():
    def __init__(self, kinematics, maxVel=1.5, k1=0.5, k2=0.5, k3=1):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger('trackingCtrl')
        self.logger.setLevel('INFO')
        self.logger.debug("Initializing TrackingController with Kinematics: {0}!".format(kinematics.name))

        # Feedback gains
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
        xk = OldX[0]  # x,k spatial
        yk = OldX[1]  # y,k spatial
        phik = OldX[2]  # phi,k spatial

        xr = ReferenceTrajectory[0]  # x,ref,k+1 spatial
        yr = ReferenceTrajectory[1]  # x,ref,k+1 spatial
        phir = ReferenceTrajectory[2]  # phi,ref,k+1 spatial

        ek = np.matrix([xk - xr, yk - yr])  ##tracking error, spatial
        eref = np.linalg.lstsq(planarRot(phir), ek, rcond=None)[
            0]  # Tracking error reference -> Result of Left array division
        erx = eref[0];  # e,x reference
        ery = eref[1];  # e,y reference
        orib = np.mod(np.mod(phik, 2 * pi) - np.mod(phir, 2 * pi), 2 * pi);  # orientation error, reference

        # Calculate actual velocities & angles
        velocity = (RefVelocity - self.k1 * abs(RefVelocity) * (erx + ery * cos(orib))) / cos(orib)
        #        if velocity > self.MaxVelocity: #saturaton on the linear velocity
        #            velocity = self.MaxVelocity
        angularVelocity = RefAngularVelocity - (
                    ((self.k2 * RefVelocity * ery) + (self.k3 * abs(RefVelocity) * tan(orib))) * (
                        cos(orib) * cos(orib)));
        return velocity.item(0), angularVelocity.item(0)
