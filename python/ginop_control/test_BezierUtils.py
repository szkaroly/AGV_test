import unittest
from BezierUtils import generateBezier, generateReferenceInput, planarRot, planarTransform

import numpy as np


class testBezierUtils(unittest.TestCase):
    # This checks a simple bezier spline start/end position & orientation
    def testBezierSplineGeneration(self):
        p1 = np.array([[0], [0]])
        p2 = np.array([[1], [0]])
        p3 = np.array([[1], [1]])
        p4 = np.array([[1], [2]])
        dt = 0.1
        time = 1
        ReferenceTrajectory = generateBezier(p1, p2, p3, p4, dt, time)
        # print(ReferenceTrajectory)
        self.assertEqual(ReferenceTrajectory[0][0], 0)
        self.assertEqual(ReferenceTrajectory[1][0], 0)
        self.assertEqual(ReferenceTrajectory[0][-1], 1)
        self.assertEqual(ReferenceTrajectory[1][-1], 2)
        self.assertAlmostEqual(ReferenceTrajectory[2][0], 0.0964, places=3)
        self.assertAlmostEqual(
            ReferenceTrajectory[2][-1], 1.5680, places=3)

    def testPlanarRotation(self):
        M1 = planarRot(90)
        self.assertAlmostEqual(M1[(0, 0)], -0.4481, places=4)
        self.assertAlmostEqual(M1[(0, 1)], -0.8940, places=4)
        self.assertAlmostEqual(M1[(1, 0)], 0.8940, places=4)
        self.assertAlmostEqual(M1[(1, 1)], -0.4481, places=4)

    def testPlanarTransform(self):
        M = planarTransform(45, np.matrix([[2], [2]]), 1.5)
        self.assertAlmostEqual(M[(0, 0)], 0.7880, places=4)
        self.assertAlmostEqual(M[(0, 1)], -1.2764, places=4)
        self.assertAlmostEqual(M[(0, 2)], 3.0000, places=4)
        self.assertAlmostEqual(M[(1, 0)], 1.2764, places=4)
        self.assertAlmostEqual(M[(1, 1)], 0.7880, places=4)
        self.assertAlmostEqual(M[(1, 2)], 3.0000, places=4)
        self.assertAlmostEqual(M[(2, 0)], 0.0000, places=4)
        self.assertAlmostEqual(M[(2, 1)], 0.0000, places=4)
        self.assertAlmostEqual(M[(2, 2)], 1.5000, places=4)

    def testGenerateReferenceInput(self):
        p1 = np.array([[0], [0]])
        p2 = np.array([[1], [0]])
        p3 = np.array([[1], [1]])
        p4 = np.array([[1], [2]])
        dt = 0.1
        time = 1
        reference_trajectory = generateBezier(p1, p2, p3, p4, dt, time)
        reference_input = generateReferenceInput(reference_trajectory, dt)
        # These values are copied from a matlab result with the same parameters
        expected = np.array([[2.4984, 2.2088, 2.0361, 2.0013, 2.0646, 2.1867, 2.3326, 2.4742, 2.5928, 2.6769, 2.7202],
                             [0.0000, 2.3509, 2.7135, 2.6807, 2.2672, 1.7252, 1.2418, 0.8651, 0.5790, 0.3553, 0.1690]])
        np.testing.assert_array_almost_equal(
            reference_input, expected, decimal=4)


unittest.main()
