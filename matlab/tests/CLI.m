classdef CLI < handle
    properties (Access = private)
        api
        defaultIterationSize = 500;
    end

    methods
        %Constructor method that receives an API object
        function self = CLI(api)
          self.api = api;
          self.api.startConnection()
        end

        %This function will command both wheels to move with v velocity
        % v - wheel rotational velocity in [rad/s]
        %TODO: Scale velocity based on wheel diameter
        function moveForward(self, v)
            self.api.setMotorVelocities(v , v );
            self.stepN(self.defaultIterationSize);
        end

        %This function will rotate the Steer to the given angle
        %steerTarget - target angle [deg]
        %TODO: Scale velocity based on wheel diameter
        function rotateSteer(self, steerTarget)
            self.api.setSteeringAngleTarget(steerTarget);
            self.stepN(self.defaultIterationSize);
        end

        %This function will move the fork to the desired height
        % 0 is the bottom
        function liftFork(self, targetPos)
            self.api.setForkMotorPositionTarget(targetPos);
            self.stepN(self.defaultIterationSize);
        end

        function stop(self)
            self.api.closeConnection();
        end
    end
    
    methods (Access = private)
      
        % This is a helper function to trigger simulation for the given
        % steps
        function stepN(self, steps)
          for i = 1:steps
              self.api.triggerStep();
          end
        end
    end

end
