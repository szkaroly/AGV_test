try:
    import vrep_norm as vrep
except Exception as e:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')
    print(e)

import logging


# This class is responsible for initiating the communication to the VREP Simulator
class vrepCommunicationAPI(object):
    def __init__(self, jointList):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger("vrepCommunicationAPI")
        self.logger.setLevel(logging.INFO)
        self.clientID = -1
        self.objects = jointList

    def initialize(self):
        '''
        Initialization, starts the connection, get the object handle for all passed object, and call object initialization on them
        '''
        self.startConnection()
        self.getObjectHandles()
        self.initializeObjects()

    def startConnection(self, ip='127.0.0.1', port=19997, synchronousMode=True):
        '''
        Connects to a defined vrep instance.
        @param ip : string : ip of the vrep instance
        @param port : int : port number for the vrep instance
        @param synchronousMode : bool : switch for the sync/async mode.
        '''
        vrep.simxFinish(-1)
        self.clientID = vrep.simxStart(ip, port, True, True, 5000, 5)  # Connect to V-REP
        if self.clientID != -1:
            self.logger.info('Connected to remote API server!')
            self.logger.info("Ip:%s\t Port:%s\tsynchronousMode:%s\tclientID:%s",
                             ip, port, synchronousMode, self.clientID)
            # enable the synchronous mode on the client:
            vrep.simxSynchronous(self.clientID, synchronousMode)
            # start the simulation:
            vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
        else:
            self.logger.error("Connection failed to VREP!")
            raise Exception("Connection Failed!")

    def closeConnection(self):
        '''
        This function gracefully shuts down the connection to the VREP instance
        '''
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
        self.logger.info("Connection closed with clientID:%s", self.clientID)

    def triggerStep(self):
        '''
        Given synchronousMode, this function triggers one iteration in VREP
        '''
        vrep.simxSynchronousTrigger(self.clientID)

    def getObjectHandles(self):
        '''
        This function calls getObjectHandle for available objects.
        '''
        for joint in self.objects:
            joint.getObjectHandle(self.clientID)

    def initializeObjects(self):
        '''
        This function calls intialize for available objectsself.
        '''
        for object in self.objects:
            object.initialize()


if __name__ == "__main__":
    from math import sin
    from vrep_Objects import *

    # Creating the objects...
    frontMotor = VelocityControlledJoint('frontMotor')
    # rightMotor = VelocityControlledJoint('rightMotor')
    # forkMotor = PositionControlledJoint('forkMotor')
    steeringMotor = PositionControlledJoint('steeringMotor')
    # Putting them into a list , that is passed to the VREP instance
    joints = [frontMotor, steeringMotor]
    # Initialize VREP communication
    VCA = vrepCommunicationAPI(joints)
    VCA.startConnection()

    # Execute the trajectory..
    for i in range(1000):
        frontMotor.setJointVelocity(sin(i / 10))
        steeringMotor.setJointPosition(0)
        print(frontMotor.getJointVelocity())
        VCA.triggerStep()
    VCA.closeConnection()
