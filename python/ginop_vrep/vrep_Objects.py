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

class AbstractObject(object):
    def __init__(self, name, typename='plainObject'):
        '''
        Handles the initialization of an Abstract Object inside VREP
        @param name -> name of the given object. This value is used for grabbing the object handle
        @param typename -> only used for logging purposes for now
        '''
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger("{0}-{1}".format(name, typename))
        self.logger.setLevel(logging.DEBUG)
        self.name = name
        self.typename = typename
        # These values shall be initialized inside the getObjectHandle function!
        self.objectHandle = -1
        self.clientID = -1

    def initialize(self):
        raise NotImplemented('initialize method has not been implemented!')


    def getObjectHandle(self, clientID):
        '''
        Gets the objectHandle from v-rep, and stores it as a member variable(self.objectHandle)
        '''
        self.clientID = clientID
        error, self.objectHandle = vrep.simxGetObjectHandle(self.clientID, self.name, vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(error, 'while retrieving Object handle for {0}'.format(self.name))

    def handleReturnValue(self, return_value, msg=''):
        '''
        Basic error handling depending on the return_code, returned by a v-rep api call
        '''
        if return_value == 0:
            return  # This is fine
        elif return_value == 1:
            print('\n')
            self.logger.debug(
                '''There is no command reply in the input buffer ->
                 Not always an error, depending on the mode its okay''' + msg)
            raise Exception('No command reply! MSG:{0}'.format(msg))
        elif return_value == 2:
            self.logger.warning('The function timed out ' + msg)
        elif return_value == 4:
            self.logger.warning(
                '''The specified operation mode is
             not supported for the given function''' + msg)
            raise Exception('''The specified operation mode is
             not supported for the given function''' + msg)
        elif return_value == 8:
            self.logger.warning(
                '''The function caused an error on the server side
                 (e.g. an invalid handle was specified)''' + msg)
            raise Exception('''The function caused an error on the server side
             (e.g. an invalid handle was specified)''' + msg)
        elif return_value == 16:
            self.logger.warning(
                '''The communication thread is still processing
             previous split command of the same type''' + msg)
            raise Exception('''The communication thread is still processing
          previous split command of the same type''' + msg)
        elif return_value == 32:
            self.logger.warning('The function caused an error on the client side' + msg)
        elif return_value == 64:
            self.logger.warning('simxStart was not yet called' + msg)


class AbstractJoint(AbstractObject):
    '''
    Abstract joint class
    '''

    def __init__(self, name, typename='abstractJoint'):
        AbstractObject.__init__(self, name, typename)
        self.jointVelocityParamID = 2012


class VelocityControlledJoint(AbstractJoint):
    '''
    Velocity controlled joint class.
    Can set the target velocity, and get back the target velocity
    '''

    def __init__(self, name):
        AbstractJoint.__init__(self, name)

    def initialize(self):
        return_code = vrep.simxSetJointTargetVelocity(self.clientID, self.objectHandle, 0,
                                                      vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(return_code, ' simxSetJointTargetVelocity first call ->{0} id:{1}'.format(self.name,
                                                                                                         self.objectHandle))
        return_code, velocity = vrep.simxGetObjectFloatParameter(self.clientID, self.objectHandle,
                                                                 self.jointVelocityParamID, vrep.simx_opmode_blocking)
        self.handleReturnValue(return_code, ' simxGetObjectFloatParameter first call ->{0} id:{1}'.format(self.name,
                                                                                                          self.objectHandle))

    def setJointVelocity(self, TargetVelocity):
        return_code = vrep.simxSetJointTargetVelocity(self.clientID, self.objectHandle, TargetVelocity,
                                                      vrep.simx_opmode_streaming)
        try:
            self.handleReturnValue(return_code,
                                   msg=' simxSetJointTargetVelocity :{0} id:{1}'.format(self.name, self.objectHandle))
        except Exception as e:
            pass

    def getJointVelocity(self):
        return_code, velocity = vrep.simxGetObjectFloatParameter(self.clientID, self.objectHandle,
                                                                 self.jointVelocityParamID, vrep.simx_opmode_blocking)
        self.handleReturnValue(return_code,
                               msg='simxGetObjectFloatParameter for {0} id:{1}'.format(self.name, self.objectHandle))
        return velocity


class PositionControlledJoint(AbstractJoint):
    '''
    Position controlled joint class. Can set the target joint position, and get back the current joint position
    '''

    def __init__(self, name):
        AbstractJoint.__init__(self, name)

    def initialize(self):
        return_code = vrep.simxSetJointTargetPosition(self.clientID, self.objectHandle, 0,
                                                      vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(return_code, ' simxSetJointTargetPosition first call ->{0}'.format(self.name))
        return_code, position = vrep.simxGetJointPosition(self.clientID, self.objectHandle,
                                                          vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(return_code, ' simxGetJointPosition first call->{0}'.format(self.name))

    def getJointPosition(self):
        '''
        @returns position : the 1 dimensional position of the joint
        '''
        return_code, position = vrep.simxGetJointPosition(self.clientID, self.objectHandle,
                                                          vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(return_code, msg=' while getting Joint position for: {0}'.format(self.name))
        return position

    def setJointPosition(self, TargetPosition):
        '''
        @param TargetPosition : target position of the joint
        '''
        return_code = vrep.simxSetJointTargetPosition(self.clientID, self.objectHandle, TargetPosition,
                                                      vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(return_code, msg='setting target position for :{0}'.format(self.name))


class DummyObject(AbstractObject):
    def __init__(self, name):
        AbstractObject.__init__(self, name, typename='DummyObject')

    def initialize(self):
        pass

    def getObjectPosition(self):
        '''
        @returns position : position array, containing the x,y,z coordinates of the object in the World coordinate frame
        '''
        return_code, position = vrep.simxGetObjectPosition(self.clientID, self.objectHandle, -1,
                                                           vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(return_code, ' -> getting object position for {0}'.format(self.name))
        return position

    def getObjectOrientation(self):
        '''
        @returns orientation : orientation array, containing the euler angles of the objectself.
        '''

        return_code, orientation = vrep.simxGetObjectOrientation(self.clientID, self.objectHandle, -1, vrep.simx_opmode_oneshot_wait)
        self.handleReturnValue(return_code, '-> getting orientation for {0}'.format(self.name))
        return orientation
