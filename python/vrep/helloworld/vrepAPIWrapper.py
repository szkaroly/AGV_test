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

import logging


# This class is responsible for initiating the communication to the VREP Simulator
class vrepCommunicationAPI(object):
    def __init__(self):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger("vrepCommunicationAPI")
        self.logger.setLevel(logging.INFO)
        self.jointVelocityParamID = 2012

        self.clientID = -1
        self.leftMotor = -1
        self.rightMotor = -1
        self.steeringMotor = -1
        self.forkMotor = -1

    def startConnection(self, ip='127.0.0.1', port=19997, synchronousMode=True):
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

    def closeConnection(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
        self.logger.info("Connection closed with clientID:%s", self.clientID)

    def triggerStep(self):
        vrep.simxSynchronousTrigger(self.clientID)

    def getObjectHandles(self):
        error, self.leftMotor = vrep.simxGetObjectHandle(
            self.clientID, 'leftMotor', vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(error)
        error, self.rightMotor = vrep.simxGetObjectHandle(
            self.clientID, 'rightMotor', vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(error)
        error, self.steeringMotor = vrep.simxGetObjectHandle(
            self.clientID, 'steeringMotor', vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(error)
        error, self.forkMotor = vrep.simxGetObjectHandle(
            self.clientID, 'forkMotor', vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(error)

    def initializeMotors(self):
        return_code = vrep.simxSetJointTargetVelocity(
            self.clientID, self.leftMotor, 0, vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(return_code)
        return_code = vrep.simxSetJointTargetVelocity(
            self.clientID, self.rightMotor, 0, vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(return_code)
        return_code = vrep.simxSetJointTargetPosition(
            self.clientID, self.steeringMotor, 0, vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(return_code)
        return_code = vrep.simxSetJointTargetPosition(
            self.clientID, self.forkMotor, 0, vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(return_code)
        return_code, position = vrep.simxGetJointPosition(
            self.clientID, self.leftMotor, vrep.simx_opmode_streaming)
        self.handleReturnValue(return_code)
        return_code, position = vrep.simxGetJointPosition(
            self.clientID, self.rightMotor, vrep.simx_opmode_streaming)
        self.handleReturnValue(return_code)
        return_code, position = vrep.simxGetJointPosition(
            self.clientID, self.steeringMotor, vrep.simx_opmode_streaming)
        self.handleReturnValue(return_code)
        return_code, position = vrep.simxGetJointPosition(
            self.clientID, self.forkMotor, vrep.simx_opmode_streaming)
        self.handleReturnValue(return_code)
        return_code, velocity = vrep.simxGetObjectFloatParameter(
            self.clientID, self.leftMotor, self.jointVelocityParamID, vrep.simx_opmode_streaming)
        self.handleReturnValue(return_code)
        return_code, velocity = vrep.simxGetObjectFloatParameter(
            self.clientID, self.rightMotor, self.jointVelocityParamID, vrep.simx_opmode_streaming)
        self.handleReturnValue(return_code)

    # Simplified API methods to handle motors easily:
    def getMotorVelocities(self):
        v_l = self.__getJointVelocity(self.leftMotor)
        v_r = self.__getJointVelocity(self.rightMotor)
        return v_l, v_r

    def setMotorVelocities(self, v_l, v_r):
        self.__setJointVeloStream(self.leftMotor, v_l)
        self.__setJointVeloStream(self.rightMotor, v_r)

    def getSteeringAngle(self):
        return_code, steering_angle = self.__getJointPositionBuffer(self.steeringMotor)
        self.handleReturnValue(return_code)
        return steering_angle

    def setSteeringAngleTarget(self, targetAngle):
        self.__setJointTargetPositionStream(self.steeringMotor, targetAngle)

    def getForkPosition(self):
        return_code, fork_position = self.__getJointPositionBuffer(self.forkMotor)
        self.handleReturnValue(return_code)
        return fork_position

    def setForkMotorPositionTarget(self, targetPosition):
        self.__setJointTargetPositionStream(self.forkMotor, targetPosition)

    # Quazi protected methods starting from here!
    def __setJointVeloStream(self, ObjectHandle, TargetVelocity):
        return_code = vrep.simxSetJointTargetVelocity(
            self.clientID, ObjectHandle, TargetVelocity, vrep.simx_opmode_streaming)
        self.handleReturnValue(return_code)

   # Set the target position for the joint ( streaming )
    def __setJointTargetPositionStream(self, ObjectHandle, TargetPosition):
        return_code = vrep.simxSetJointTargetPosition(
            self.clientID, ObjectHandle, TargetPosition, vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(return_code)

    def __getJointVelocity(self, ObjectHandle):
        return_code, velocity = vrep.simxGetObjectFloatParameter(
            self.clientID, ObjectHandle, self.jointVelocityParamID, vrep.simx_opmode_buffer)
        self.handleReturnValue(return_code)
        return velocity

    def __getJointPositionBuffer(self, ObjectHandle):
        # __getJointPositionBuffer gets the Passed handle's joint
        # position, must call "getJointPositionFirstCall" first
        return_code, position = vrep.simxGetJointPosition(
            self.clientID, ObjectHandle, vrep.simx_opmode_buffer)
        self.handleReturnValue(return_code)
        return position

    def __getObjectVelocityFirstCall(self, ObjectHandle):
        # getObjectVelocityFirstCall gets the Passed handle's
        # velocity, must call this before "getObjectVelocityBuffer"
        return_code, linear_velocity, angular_velocity = vrep.simxGetObjectVelocity(
            self.clientID, ObjectHandle, vrep.simx_opmode_streaming)
        self.handleReturnValue(return_code)
        return linear_velocity, angular_velocity

    def __getObjectVelocityBuffer(self, ObjectHandle):
        # getObjectVelocityBuffer gets the Passed handle's
        # velocity, must call "getObjectVelocityFirstCall" first
        return_code, linear_velocity, angular_velocity = vrep.simxGetObjectVelocity(
            self.clientID, ObjectHandle, vrep.simx_opmode_buffer)
        self.handleReturnValue(return_code)
        return linear_velocity, angular_velocity

    def handleReturnValue(self, return_value):
        if return_value == 0:
            return  # This is fine
        elif return_value == 1:
            self.logger.debug(
                '''There is no command reply in the input buffer ->
                 Not always an error, deping on the mode its okay''')
        elif return_value == 2:
            self.logger.warning('The function timed out ')
        elif return_value == 4:
            self.logger.warning(
                '''The specified operation mode is
             not supported for the given function''')
        elif return_value == 8:
            self.logger.warning(
                '''The function caused an error on the server side
                 (e.g. an invalid handle was specified)''')
        elif return_value == 16:
            self.logger.warning(
                '''The communication thread is still processing
             previous split command of the same type''')
        elif return_value == 32:
            self.logger.warning('The function caused an error on the client side')
        elif return_value == 64:
            self.logger.warning('simxStart was not yet called')


if __name__ == "__main__":
    VCA = vrepCommunicationAPI()
    VCA.startConnection()
    VCA.setMotorVelocities(10, 10)
    for i in range(1000):
        VCA.triggerStep()
    VCA.closeConnection()
