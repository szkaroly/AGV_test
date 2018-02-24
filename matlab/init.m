API = vrepApiWrapper;
API.startConnection('127.0.0.1', 19997);
assert(API.clientID ~= -1, 'Connection failed, aborting program');
running = true;
%Enter the control loop
while(running)
  %Get Velocity
  [vl, vr] = API.getMotorVelocities();
  asd = API.getJointPositionBuffer(API.leftMotor)
  pos = API.getSteeringAngle();
  %Calculate position

  %Calculate command

  %Calculate control

  %Send Velocity to HW
  vl_cmd = 0.5;
  vr_cmd = -0.5;
  API.setMotorVelocities(vl_cmd,vr_cmd);
  API.setSteeringAngleTarget(1.56);
  API.triggerStep();
end

API.closeConnection();