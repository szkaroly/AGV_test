try:
    import vrep
except:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')

import time
import sys
import logging


class vrepCommunicationAPI(object):
    def __init__(self, ip='127.0.0.1', port=19997, synchronousMode=True):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger("vrepCommunicationAPI")
        self.logger.setLevel(logging.INFO)

        vrep.simxFinish(-1)
        self.clientID = vrep.simxStart(ip, port, True, True, 5000, 5)  # Connect to V-REP
        if self.clientID != -1:
            self.logger.info('Connected to remote API server!')
            self.logger.info("Ip:{0}\t Port:{1}\tsynchronousMode:{2}\tclientID:{3}".format(
                ip, port, synchronousMode, self.clientID))
            # enable the synchronous mode on the client:
            vrep.simxSynchronous(self.clientID, synchronousMode)
            # start the simulation:
            vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
        else:
            self.logger.error("Connection failed to VREP!")

    def closeConnection(self):
        self.vrep.simxStopSimulation(self.clientID, self.vrep.simx_opmode_oneshot_wait)

    def triggerStep(self):
        vrep.simxSynchronousTrigger(self.clientID)

    def getObjectHandles(self):
        error, self.leftMotor = self.vrep.simxGetObjectHandle(
            self.clientID, 'leftMotor', self.vrep.simx_opmode_oneshot_wait)
        error, self.rightMotor = self.vrep.simxGetObjectHandle(
            self.clientID, 'rightMotor', self.vrep.simx_opmode_oneshot_wait)
        error, self.steeringMotor = self.vrep.simxGetObjectHandle(
            self.clientID, 'steeringMotor', self.vrep.simx_opmode_oneshot_wait)
        error, self.forkMotor = self.vrep.simxGetObjectHandle(
            self.clientID, 'forkMotor', self.vrep.simx_opmode_oneshot_wait)

    def initializeMotors(self):
        returnCode = self.vrep.simxSetJointTargetVelocity(
            self.clientID, self.leftMotor,     0, self.vrep.simx_opmode_oneshot_wait)
        returnCode = self.vrep.simxSetJointTargetVelocity(
            self.clientID, self.rightMotor,    0, self.vrep.simx_opmode_oneshot_wait)
        returnCode = self.vrep.simxSetJointTargetPosition(
            self.clientID, self.steeringMotor, 0, self.vrep.simx_opmode_oneshot_wait)
        returnCode = self.vrep.simxSetJointTargetPosition(
            self.clientID, self.forkMotor, 0, self.vrep.simx_opmode_oneshot_wait)
        returnCode, position = self.vrep.simxGetJointPosition(
            self.clientID, self.leftMotor, self.vrep.simx_opmode_streaming)
        returnCode, position = self.vrep.simxGetJointPosition(
            self.clientID, self.rightMotor, self.vrep.simx_opmode_streaming)
        returnCode, position = self.vrep.simxGetJointPosition(
            self.clientID, self.steeringMotor, self.vrep.simx_opmode_streaming)
        returnCode, position = self.vrep.simxGetJointPosition(
            self.clientID, self.forkMotor, self.vrep.simx_opmode_streaming)
        returnCode, velocity = self.vrep.simxGetObjectFloatParameter(
            self.clientID, self.leftMotor, self.jointVelocityParamID, self.vrep.simx_opmode_streaming)
        returnCode, velocity = self.vrep.simxGetObjectFloatParameter(
            self.clientID, self.rightMotor, self.jointVelocityParamID, self.vrep.simx_opmode_streaming)

    # Simplified API methods to handle motors easily:
    def getMotorVelocities(self):
        vl = self.__getJointVelocity(self.leftMotor)
        vr = self.__getJointVelocity(self.rightMotor)

    def setMotorVelocities(self, vl, vr):
        self.__setJointVeloStream(self.leftMotor, vl)
        self.__setJointVeloStream(self.rightMotor, vr)

    #
    def getSteeringAngle(self):
        [returnCode, steeringAngle] = self.__getJointPositionBuffer(self.steeringMotor)
        self.handleReturnValue(returnCode)

    def setSteeringAngleTarget(self, targetAngle):
        self.__setJointTargetPositionStream(self.steeringMotor, targetAngle)

    def getForkPosition(self):
        returnCode, forkPosition = self.__getJointPositionBuffer(self.forkMotor)
        self.handleReturnValue(returnCode)
        return forkPosition

    def setForkMotorPositionTarget(self, targetPosition):
        self.__setJointTargetPositionStream(self.forkMotor, targetPosition)

    def __setJointVeloStream(self, ObjectHandle, TargetVelocity):
        # setJointVeloOneShot  set Joint velocity repeatedly
        #   Call arguments: (ObjectHandle , TargetVelocity)
        returnCode = self.vrep.simxSetJointTargetVelocity(
            self.clientID, ObjectHandle, TargetVelocity, self.vrep.simx_opmode_streaming)
        self.handleReturnValue(returnCode)

   # Set the target position for the joint ( streaming )
    def __setJointTargetPositionStream(self, ObjectHandle, TargetPosition):
        returnCode = self.vrep.simxSetJointTargetPosition(
            self.clientID, ObjectHandle, TargetPosition, self.vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(returnCode)

    def __getJointVelocity(self, ObjectHandle):
        # clientID: the client ID. refer to simxStart.
        # objectHandle: handle of the object
        # parameterID: identifier of the parameter to retrieve. See the list of all possible object parameter identifiers
        # parameterValue: pointer to a location that will receive the value of the parameter
        # operationMode: a remote API def  operation mode. Recommed operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls), or simx_opmode_blocking (deping on the inted usage)
        returnCode, velocity = self.vrep.simxGetObjectFloatParameter(
            self.clientID, ObjectHandle, self.jointVelocityParamID, self.vrep.simx_opmode_buffer)
        self.handleReturnValue(returnCode)
        return velocity

    def __getJointPositionBuffer(self, ObjectHandle):
        # __getJointPositionBuffer gets the Passed handle's joint
        # position, must call "getJointPositionFirstCall" first
        returnCode, position = self.vrep.simxGetJointPosition(
            self.clientID, ObjectHandle, self.vrep.simx_opmode_buffer)
        self.handleReturnValue(returnCode)
        return position

    def __getObjectVelocityFirstCall(self, ObjectHandle):
        # getObjectVelocityFirstCall gets the Passed handle's
        # velocity, must call this before "getObjectVelocityBuffer"
        returnCode, linearVelocity, angularVelocity = self.vrep.simxGetObjectVelocity(
            self.clientID, ObjectHandle, self.vrep.simx_opmode_streaming)
        return linearVelocity, angularVelocity

    def __getObjectVelocityBuffer(self, ObjectHandle):
        # getObjectVelocityBuffer gets the Passed handle's
        # velocity, must call "getObjectVelocityFirstCall" first
        returnCode, linearVelocity, angularVelocity = self.vrep.simxGetObjectVelocity(
            self.clientID, ObjectHandle, simx_opmode_buffer)
        return linearVelocity, angularVelocity

    def handleReturnValue(self, returnValue):
        if returnValue == 0:
            return  # This is fine
        elif returnValue == 1:
            self.logger.debug(
                'There is no command reply in the input buffer -> Not always an error, deping on the mode its okay')
        elif returnValue == 2:
            self.logger.warning('The function timed out ')
        elif returnValue == 4:
            self.logger.warning('The specified operation mode is not supported for the given function')
        elif returnValue == 8:
            self.logger.warning(
                'The function caused an error on the server side (e.g. an invalid handle was specified)')
        elif returnValue == 16:
            self.logger.warning('The communication thread is still processing previous split command of the same type')
        elif returnValue == 32:
            self.logger.warning('The function caused an error on the client side')
        elif returnValue == 64:
            self.logger.warning('simxStart was not yet called')


if __name__ == "__main__":
    vca = vrepCommunicationAPI()
