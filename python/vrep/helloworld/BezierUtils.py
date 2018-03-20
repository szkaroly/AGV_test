import numpy as np
from math import atan2
def generateBezier(P1,P2,P3,P4,SamplingTime,EndTime):
    t = np.arange(0,EndTime+SamplingTime,SamplingTime)
    t = t/t[-1]
    ReferencePosition = np.kron(np.power((1-t),3),P1)+  np.kron(3*np.power((1-t), 2)*t,P2) + np.kron(3*np.power((1-t)*t,2), P3) + np.kron(np.power(t,3),P4)
    print(ReferencePosition.shape)
    print(ReferencePosition)
    diff = np.diff(ReferencePosition)
    ReferenceOrientation = np.arctan2(diff[0],diff[1])
    ReferenceOrientation[-1] = ReferenceOrientation[-2]
    print('ReferenceOrientation:', ReferenceOrientation)
    #the last two orientations are identical (this is just an approximation, but
    # it is needed to avoid decreasing the number of samples)
    return ReferencePosition, ReferenceOrientation



#TODO : Write tests
if __name__ == "__main__":
    p1 = np.array([[0], [0]])
    p2 = np.array([[1], [1]])
    p3 = np.array([[2], [2]])
    p4 = np.array([[2], [1]])

    print('p1shape',p1.shape)
    print('p1shape',p2.shape)
    print('p1shape',p3.shape)
    print('p1shape',p4.shape)
    generateBezier(p1,p2,p3,p4,0.01,1)
