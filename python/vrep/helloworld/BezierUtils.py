import numpy as np
from math import cos, sin, atan2

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

def generateReferenceInput (ReferenceTrajectory, SamplingTime):
    XLIMIT = 0.00001
    N = len(ReferenceTrajectory[1])

    for ii in range(N-1):
        if ii == 1:
            phik = ReferenceTrajectory[1][ii]
        else:
            phik = ReferenceTrajectory[1][ii-1]
    xk = ReferenceTrajectory[0][0][ii]; #x,ref,k spatial
    yk = ReferenceTrajectory[0][1][ii]; #y,ref,k spatial
    xs = ReferenceTrajectory[0][0][ii+1]; #x,ref,k+1 spatial
    ys = ReferenceTrajectory[0][1][ii+1]; #y,ref,k+1 spatial
    inverz = np.linalg.inv(planarTransform(phik,np.matrix([[xk],[yk]]),1))
    DBody = inverz*np.matrix([[xs],[ys], [1]])
    print('DBODY:\n', DBody)
    xb = DBody[0]
    yb = DBody[1]

    if abs(xb) < XLIMIT:
        RefVel = (0,0)
    else:
        #FIXME
        helper = np.matrix([[xb*cos(phik)-yb*sin(phik), xb*sin(phik)+yb*cos(phik)],
                           [-sin(phik)                , cos(phik)]])
        RefVel = 1 / (SamplingTime*xb) * helper * np.matrix([xs-xk,ys-yk])
# RefVel = 1/(SamplingTime*xb)*[xb*cos(phik)-yb*sin(phik) xb*sin(phik)+yb*cos(phik);
#                               -sin(phik) cos(phik)]*[xs-xk;ys-yk];

        print(RefVel)

#Planar rotation - checked
def planarRot(angle):
    #this is a 2x2 matrix
    return np.matrix([[cos(angle),-sin(angle)],[sin(angle), cos(angle)]])


# Planar Transform - checked
def planarTransform(angle,position,scale):
    #new matrix is constructed via: [2x2 + 2x1]
    #                               [0,  0, 1 ]
    matrix = np.hstack([planarRot(angle), position])
    matrix = np.vstack([matrix, np.matrix([0,0,1])])
    #matrix = np.matrix([planarRot(angle), position, np.matrix([0,0,1])])
    return scale*matrix



#TODO : Write tests
if __name__ == "__main__":
    def test1():
        p1 = np.array([[0], [0]])
        p2 = np.array([[1], [1]])
        p3 = np.array([[2], [2]])
        p4 = np.array([[2], [1]])

        print('p1shape',p1.shape)
        print('p1shape',p2.shape)
        print('p1shape',p3.shape)
        print('p1shape',p4.shape)
        referenceTraj = generateBezier(p1,p2,p3,p4,0.01,1)
        #print('length is :' , len(referenceTraj[1]))
        return referenceTraj
    def test2():
        angle = 0
        position = np.matrix([[1],[1]])
        print(planarTransform(angle, position, 6))

    def test3():
        traj = test1()
        generateReferenceInput(traj, 0.01)
    test3()
