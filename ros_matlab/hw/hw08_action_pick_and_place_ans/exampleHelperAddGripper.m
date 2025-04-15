function outputRobot = exampleHelperAddGripper(inputRobot)
%--------------------------------------------------------------------------
% Provided by competition organizers to attaches Robotiq EPick gripper and 
% IO Coupling to the UR5e robot
% 
% Inputs:
% inputRobot (rigidBodyTree) - of the ur5e 
%
% Output:
% Updated robot with Bellow as the tip the robot.
%
% Copyright 2023 The MathWorks, Inc.
% Edited by Juan Rojas on 3/23/2024
%--------------------------------------------------------------------------

% Add I/O Coupling
coupling = rigidBody('robotic_coupler','MaxNumCollisions',1);
tf = eul2tform([0 0 0]);
tf(:,4) = [0; 0; 0.004; 1]; % To avoid collision
addCollision(coupling,'cylinder',[0.083/2 0.0139],tf);
couplingJoint = rigidBodyJoint('robotic_coupler_joint','fixed');
coupling.Joint = couplingJoint;
curEndEffectorBodyName = inputRobot.BodyNames{10};
addBody(inputRobot,coupling,curEndEffectorBodyName);

% Add Gripper Unit
transformGripper = eul2tform([0 0 0]);
transformGripper(:,4) = [0; 0; 0.0139; 1]; % The width is 16.9 or 13.9
gripper = rigidBody('EPick','MaxNumCollisions',1);
tf = eul2tform([0 0 0]);
tf(:,4) = [0; 0; 0.045; 1]; % Gripper Width
addCollision(gripper,'cylinder',[0.083/2 0.1023],tf);
gripperJoint = rigidBodyJoint('gripperJoint','fixed');
gripper.Joint = gripperJoint;
setFixedTransform(gripper.Joint, transformGripper);
curEndEffectorBodyName = inputRobot.BodyNames{11};
addBody(inputRobot,gripper,curEndEffectorBodyName);

% Add Extention tube
transformTube = eul2tform([0 0 0]);
transformTube(:,4) = [0; 0; 0.165; 1]; 
tube = rigidBody('Tube','MaxNumCollisions',1);
tf = eul2tform([0 0 0]);
tf(:,4) = [0; 0; 0.101; 1]; % Gripper Width
addCollision(tube,'cylinder',[0.005 0.2],tf);
tubeJoint = rigidBodyJoint('gripper_tip_link','fixed');
tube.Joint = tubeJoint;
setFixedTransform(tube.Joint, transformTube);
curEndEffectorBodyName = inputRobot.BodyNames{12};
addBody(inputRobot,tube,curEndEffectorBodyName);

outputRobot = inputRobot;
end