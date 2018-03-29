import numpy as np
from math import cos, sin, atan2
import logging

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)


'''
P1 : initial position of the desired path
    P2 : initial tangent of the desired path
    P3 : final position of the desired path
    P4 : final tangent of the desired path
    SamplingTime : timestep in seconds
    EndTime : total simulation time
''' # Checked
def generateBezier(P1: np.array, P2: np.array, P3: np.array, P4: np.array,
                   SamplingTime: float, EndTime: float) -> np.array:
    t = np.arange(start = 0, stop = EndTime + 2 * SamplingTime, step = SamplingTime)
    t = t / t[-1] # ok so far

    ReferencePosition =   np.kron(    np.power((1 - t), 3)    , P1)\
                        + np.kron(3 * np.power((1 - t), 2) * t, P2)\
                        + np.kron(3 * (1 - t) * np.power(t,2) , P3)\
                        + np.kron(    np.power(t, 3)          , P4)

    diff = np.diff(ReferencePosition)
    ReferenceOrientation = np.arctan2(diff[1], diff[0])
    ReferenceOrientation = np.append(ReferenceOrientation, ReferenceOrientation[-1])
    # the last two orientations are identical (this is just an approximation, but
    # it is needed to avoid decreasing the number of samples)
    return np.vstack([ReferencePosition[0], ReferencePosition[1], ReferenceOrientation])

'''
This function generates the reference velocity input (v, omega) for the vehicle
ReferenceTrajectory : np.array  -> containing x,y, orientation values
SamplingTime        : float     -> timestep
XLIMIT              : flaot     -> ???
'''
def generateReferenceInput(ReferenceTrajectory : np.array, SamplingTime : float , XLIMIT = 0.00001) -> np.array:
    N = len(ReferenceTrajectory[1])
    reference_input = np.empty([2,N-1])
    for ii in range(N - 1):
        if ii == 0:
            phik = ReferenceTrajectory[(2,ii)]
        else:
            phik = ReferenceTrajectory[(2,ii - 1)]
        xk = ReferenceTrajectory[(0,ii)]   # x,ref,k spatial
        yk = ReferenceTrajectory[(1,ii)]    # y,ref,k spatial
        xs = ReferenceTrajectory[(0,ii+1)]  # x,ref,k+1 spatial
        ys = ReferenceTrajectory[(1,ii+1)]  # y,ref,k+1 spatial
        planarT = planarTransform(phik, np.matrix([[xk],[yk]]), 1)
        inverz = np.linalg.inv(planarT)
        DBody = inverz * np.matrix([[xs], [ys], [1]])
        xb = float(DBody[0])
        yb = float(DBody[1])

        if abs(xb) < XLIMIT:
            RefVel = (0, 0)
        else:
            helper = np.matrix([[(xb * cos(phik) - yb * sin(phik)), (xb * sin(phik) + yb * cos(phik))],
                                [-sin(phik)                     , cos(phik)]])
            RefVel = 1 / (SamplingTime * xb) * helper * np.matrix([[xs - xk], [ys - yk]])

        reference_input[(0,ii)] = RefVel[0]
        reference_input[(1,ii)] = RefVel[1]

    return reference_input




# Planar rotation - checked
def planarRot(angle):
    # this is a 2x2 matrix
    return np.matrix([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])


# Planar Transform - checked
def planarTransform(angle, position, scale):
    # new matrix is constructed via: [2x2 + 2x1]
    #                               [0,  0, 1 ]
    matrix = np.hstack([planarRot(angle), position])
    result = np.vstack([matrix, np.matrix([0, 0, 1])])
    # matrix = np.matrix([planarRot(angle), position, np.matrix([0,0,1])])
    return scale * result


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
            dt = 0.1
            time = 1
            ReferenceTrajectory = generateBezier(p1, p2, p3, p4, dt, time)
            #print(ReferenceTrajectory)
            self.assertEqual(ReferenceTrajectory[0][0], 0)
            self.assertEqual(ReferenceTrajectory[1][0], 0)
            self.assertEqual(ReferenceTrajectory[0][-1], 1)
            self.assertEqual(ReferenceTrajectory[1][-1], 2)
            self.assertAlmostEqual(ReferenceTrajectory[2][0], 0.0964, places = 3)
            self.assertAlmostEqual(ReferenceTrajectory[2][-1], 1.5680, places = 3)

        def testPlanarRotation(self):
            M1 = planarRot(90)
            self.assertAlmostEqual(M1[(0,0)], -0.4481, places = 4)
            self.assertAlmostEqual(M1[(0,1)], -0.8940, places = 4)
            self.assertAlmostEqual(M1[(1,0)],  0.8940, places = 4)
            self.assertAlmostEqual(M1[(1,1)], -0.4481, places = 4)


        def testPlanarTransform(self):
            M  = planarTransform(45, np.matrix([[2], [2]]), 1.5)
            self.assertAlmostEqual(M[(0,0)],  0.7880, places = 4)
            self.assertAlmostEqual(M[(0,1)], -1.2764, places = 4)
            self.assertAlmostEqual(M[(0,2)],  3.0000, places = 4)
            self.assertAlmostEqual(M[(1,0)],  1.2764, places = 4)
            self.assertAlmostEqual(M[(1,1)],  0.7880, places = 4)
            self.assertAlmostEqual(M[(1,2)],  3.0000, places = 4)
            self.assertAlmostEqual(M[(2,0)],  0.0000, places = 4)
            self.assertAlmostEqual(M[(2,1)],  0.0000, places = 4)
            self.assertAlmostEqual(M[(2,2)],  1.5000, places = 4)

        def testGenerateReferenceInput(self):
            p1 = np.array([[0], [0]])
            p2 = np.array([[1], [0]])
            p3 = np.array([[1], [1]])
            p4 = np.array([[1], [2]])
            dt = 0.1
            time = 1
            reference_trajectory = generateBezier(p1, p2, p3, p4, dt, time)
            reference_input = generateReferenceInput(reference_trajectory, dt)
            #These values are copied from a matlab result with the same parameters
            expected = np.array([[2.4984, 2.2088, 2.0361, 2.0013, 2.0646, 2.1867, 2.3326, 2.4742, 2.5928, 2.6769, 2.7202],
                                 [0.0000, 2.3509, 2.7135, 2.6807, 2.2672, 1.7252, 1.2418, 0.8651, 0.5790, 0.3553, 0.1690]])
            np.testing.assert_array_almost_equal(reference_input, expected, decimal=4)


    unittest.main()
