StartPosition = [ 1; 1];
InitialTangent = [ 2; 2];
EndPosition = [2;4];
EndTangent = [1;3];

scatter(StartPosition(1), StartPosition(2))
hold on;
scatter(InitialTangent(1), InitialTangent(2))
scatter(EndPosition(1), EndPosition(2))
scatter(EndTangent(1), EndTangent(2))


EndTime = 20; %sec %final time of the simulation
SamplingTime = 0.100; %sec %sampling time

MaxVelocity = 10; %maximal linear velocity of the car, m/s

ReferenceTrajectory = GenerateBezier(StartPosition,InitialTangent,EndTangent,EndPosition,SamplingTime,EndTime);
plot(ReferenceTrajectory(1,:),ReferenceTrajectory(2,:),'r','Linewidth',1.5)
csvwrite('referencetrajectory.csv', ReferenceTrajectory')

ReferenceInputs = GenerateReferenceInput(ReferenceTrajectory,SamplingTime);
csvwrite('referencenputs.csv', ReferenceInputs')



t = [0:SamplingTime:EndTime]
