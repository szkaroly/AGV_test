%Create api to the robot
API = vrepApiWrapper;
%Pass it to the command line interface object
cli = CLI(API);

%Now the connection is up! The following commands are available from the command line:
%% Example commands:
%% To move the wheels in a given direction with the velocity [rad/s] (negatíve value will rotate backwards) :
% cli.moveForward(v)
%% To move the steering to a given angle [deg]
% cli.rotateSteer(targetAng)
%% To move the fork to a desired height [meters]:
% cli.liftFork(pos)
%% To disconnect:
% cli.stop()