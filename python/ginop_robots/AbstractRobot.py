import logging
class AbstractRobot(object):
    def __init__(self, TrackingController, HardwareAbstractionLayer, name='robot'):
        FORMAT = '[%(asctime)-15s][%(levelname)s][%(funcName)s] %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger(name)
        self.logger.setLevel('INFO')
        self.logger.debug("Initializing {0}!".format(name))
        self.TrackingController = TrackingController
        self.hal = HardwareAbstractionLayer
        self.hal.initialize()
        self.logger.info("{0} instance has been initialized!".format(name))

    def shutdown(self):
        self.hal.closeConnection()
