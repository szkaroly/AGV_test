import numpy as np
from math import cos, sin, atan2
import pandas as pd


''' P1 : initial position of the desired path
    P2 : initial tangent of the desired path
    P3 : final position of the desired path
    P4 : final tangent of the desired path
    SamplingTime : timestep in seconds
    EndTime : total simulation time'''
def generateBezier(P1: np.array, P2: np.array, P3: np.array, P4: np.array,
                   SamplingTime: float, EndTime: float):
    t = np.arange(start = 0, stop = EndTime + 2 * SamplingTime, step = SamplingTime)
    t = t / t[-1] # ok so far

    ReferencePosition =   np.kron(    np.power((1 - t), 3)    , P1)\
                        + np.kron(3 * np.power((1 - t), 2) * t, P2)\
                        + np.kron(3 * (1 - t) * np.power(t,2) , P3)\
                        + np.kron(    np.power(t, 3)          , P4)

    print('Refpos', ReferencePosition)

    diff = np.diff(ReferencePosition)
    print('\ndiff[0]->',diff[0])
    print('\ndiff[1]->',diff[1], '\n\n')
    ReferenceOrientation = np.arctan2(diff[1], diff[0])
    ReferenceOrientation = np.append(ReferenceOrientation, ReferenceOrientation[-1])
    # the last two orientations are identical (this is just an approximation, but
    # it is needed to avoid decreasing the number of samples)
    return np.vstack([ReferencePosition[0], ReferencePosition[1], ReferenceOrientation])


def generateReferenceInput(ReferenceTrajectory, SamplingTime):
    XLIMIT = 0.00001
    N = len(ReferenceTrajectory[1])

    for ii in range(N - 1):
        if ii == 1:
            phik = ReferenceTrajectory[1][ii]
        else:
            phik = ReferenceTrajectory[1][ii - 1]
    xk = ReferenceTrajectory[0][0][ii];  # x,ref,k spatial
    yk = ReferenceTrajectory[0][1][ii];  # y,ref,k spatial
    xs = ReferenceTrajectory[0][0][ii + 1];  # x,ref,k+1 spatial
    ys = ReferenceTrajectory[0][1][ii + 1];  # y,ref,k+1 spatial
    inverz = np.linalg.inv(planarTransform(phik, np.matrix([[xk], [yk]]), 1))
    DBody = inverz * np.matrix([[xs], [ys], [1]])
    print('DBODY:\n', DBody)
    xb = DBody[0]
    yb = DBody[1]

    if abs(xb) < XLIMIT:
        RefVel = (0, 0)
    else:
        # FIXME
        helper = np.matrix([[xb * cos(phik) - yb * sin(phik), xb * sin(phik) + yb * cos(phik)],
                            [-sin(phik), cos(phik)]])
        RefVel = 1 / (SamplingTime * xb) * helper * np.matrix([xs - xk, ys - yk])
        # RefVel = 1/(SamplingTime*xb)*[xb*cos(phik)-yb*sin(phik) xb*sin(phik)+yb*cos(phik);
        #                               -sin(phik) cos(phik)]*[xs-xk;ys-yk];

        print(RefVel)


# Planar rotation - checked
def planarRot(angle):
    # this is a 2x2 matrix
    return np.matrix([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])


# Planar Transform - checked
def planarTransform(angle, position, scale):
    # new matrix is constructed via: [2x2 + 2x1]
    #                               [0,  0, 1 ]
    matrix = np.hstack([planarRot(angle), position])
    matrix = np.vstack([matrix, np.matrix([0, 0, 1])])
    # matrix = np.matrix([planarRot(angle), position, np.matrix([0,0,1])])
    return scale * matrix


# TODO : Write tests
if __name__ == "__main__":
    import unittest
    class testBezierUtils(unittest.TestCase):
        #This checks a simple bezier spline start/end position & orientation
        def testBezierSplineGeneration(self):
            p1 = np.array([[0], [0]])
            p2 = np.array([[1], [0]])
            p3 = np.array([[1], [1]])
            p4 = np.array([[1], [2]])
            ReferenceTrajectory = generateBezier(p1, p2, p3, p4, 0.1, 1)
            #print(ReferenceTrajectory)
            self.assertEqual(ReferenceTrajectory[0][0], 0)
            self.assertEqual(ReferenceTrajectory[1][0], 0)
            self.assertEqual(ReferenceTrajectory[0][-1], 1)
            self.assertEqual(ReferenceTrajectory[1][-1], 2)
            self.assertAlmostEqual(ReferenceTrajectory[2][0], 0.0964, places = 3)
            self.assertAlmostEqual(ReferenceTrajectory[2][-1], 1.5680, places = 3)




            def test2(self):
                angle = 0
                position = np.matrix([[1], [1]])
                print(planarTransform(angle, position, 6))


                def test3(self):
                    pass
                    #traj = test1()
                    #generateReferenceInput(traj, 0.01)


    unittest.main()
