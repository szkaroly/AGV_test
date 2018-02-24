clc; clear all;
API = vrepApiWrapper;
API.startConnection('127.0.0.1', 19997);

tolerance = 0.01;

assert(API.clientID ~= -1, 'Connection not succesful, is the simulation running?')
assert(API.rightMotor ~= -1);
assert(API.leftMotor ~= -1);
assert(API.steeringJoint ~= -1);

display('Set velocities to zero')
API.setMotorVelocities(0,0);
API.setSteeringAngleTarget(0);
for i = 1:100
 API.triggerStep();
end
[vl, vr] = API.getMotorVelocities();
[steeringPos] = API.getSteeringAngle();

assert(abs(vl) < tolerance, 'Could not slow down the left wheel')
assert(abs(vr) < tolerance, 'Could not slow down the right wheel')
assert(abs(steeringPos) < tolerance, 'Could not set steering position to zero')

display('Set motor velocities');
targetVel = 0.5;
API.setMotorVelocities(targetVel,targetVel);

for i = 1:500
 API.triggerStep();
end

[vl, vr] = API.getMotorVelocities();
assert(abs(vl - targetVel) < tolerance, 'Could not speed up the left wheel')
assert(abs(vr - targetVel) < tolerance, 'Could not speed up the right wheel')


display('Set velocities to zero')
API.setMotorVelocities(0,0);
API.setSteeringAngleTarget(1.57);

for i = 1:200
 API.triggerStep();
end
[vl, vr] = API.getMotorVelocities();

steeringAng = API.getSteeringAngle();
assert(abs(vl) < tolerance, 'Could not slow down the left wheel')
assert(abs(vr) < tolerance, 'Could not slow down the right wheel')
assert(abs(steeringAng - 1.57) < tolerance, 'Could not turn the steering joint to 90 deg ') 

API.closeConnection();
display('Test passed! Motors reached  the commanded velocity then stopped.')
