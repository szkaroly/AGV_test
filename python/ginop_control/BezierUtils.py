import numpy as np
from math import cos, sin



def generateBezier(P1: np.array, P2: np.array, P3: np.array, P4: np.array,
                   SamplingTime: float, EndTime: float) -> np.array:
    """
    P1 : initial position of the desired path
    P2 : initial tangent of the desired path
    P3 : final position of the desired path
    P4 : final tangent of the desired path
    SamplingTime : timestep in seconds
    EndTime : total simulation time
    """

    t = np.arange(start=0, stop=EndTime + 2 * SamplingTime, step=SamplingTime)
    t = t / t[-1]  # ok so far

    ReferencePosition = np.kron(np.power((1 - t), 3), P1)\
        + np.kron(3 * np.power((1 - t), 2) * t, P2)\
        + np.kron(3 * (1 - t) * np.power(t, 2), P3)\
        + np.kron(np.power(t, 3), P4)

    diff = np.diff(ReferencePosition)
    ReferenceOrientation = np.arctan2(diff[1], diff[0])
    ReferenceOrientation = np.append(
        ReferenceOrientation, ReferenceOrientation[-1])
    # the last two orientations are identical (this is just an approximation, but
    # it is needed to avoid decreasing the number of samples)
    return np.vstack([ReferencePosition[0], ReferencePosition[1], ReferenceOrientation])


def generateReferenceInput(ReferenceTrajectory: np.array, SamplingTime: float, XLIMIT=0.00001) -> np.array:
    """
    This function generates the reference velocity input (v, omega) for the vehicle
    ReferenceTrajectory : np.array  -> containing x,y, orientation values
    SamplingTime        : float     -> timestep
    XLIMIT              : float     -> ???
    """
    N = len(ReferenceTrajectory[1])
    reference_input = np.empty([2, N - 1])
    for ii in range(N - 1):
        if ii == 0:
            phik = ReferenceTrajectory[(2, ii)]
        else:
            phik = ReferenceTrajectory[(2, ii - 1)]
        xk = ReferenceTrajectory[(0, ii)]   # x,ref,k spatial
        yk = ReferenceTrajectory[(1, ii)]    # y,ref,k spatial
        xs = ReferenceTrajectory[(0, ii + 1)]  # x,ref,k+1 spatial
        ys = ReferenceTrajectory[(1, ii + 1)]  # y,ref,k+1 spatial
        planarT = planarTransform(phik, np.matrix([[xk], [yk]]), 1)
        inverz = np.linalg.inv(planarT)
        DBody = inverz * np.matrix([[xs], [ys], [1]])
        xb = float(DBody[0])
        yb = float(DBody[1])

        if abs(xb) < XLIMIT:
            RefVel = (0, 0)
        else:
            helper = np.matrix([[(xb * cos(phik) - yb * sin(phik)), (xb * sin(phik) + yb * cos(phik))],
                                [-sin(phik), cos(phik)]])
            RefVel = 1 / (SamplingTime * xb) * helper * \
                np.matrix([[xs - xk], [ys - yk]])

        reference_input[(0, ii)] = RefVel[0]
        reference_input[(1, ii)] = RefVel[1]
    return reference_input


def planarRot(angle):
    ''' Does a planar rotation'''
    # this is a 2x2 matrix
    return np.matrix([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])


def planarTransform(angle, position, scale):
    ''' Does a planar transformation on the position '''
    # new matrix is constructed via: [2x2 + 2x1]
    #                                [0,  0, 1 ]
    matrix = np.hstack([planarRot(angle), position])
    result = np.vstack([matrix, np.matrix([0, 0, 1])])
    return scale * result


if __name__ == "__main__":
    import unittest
