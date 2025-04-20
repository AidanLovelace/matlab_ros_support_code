function [zoneComplete] = PickandPlaceARMChallenge(zoneInspect, optns)   
%--------------------------------------------------------------------------
% PickandPlaceArmChallenge
% Divides the workspace into different zones according to Robocup
% regulations.
%
% Strategy seeks to position the manipulator at an advantageous
% configuration from which to take images for object identification and
% subsequent pose estimation.
%
% After identifying one object or objects, pick and place them.
%
% Note:
% This strategy can be improved in multiple ways. Are start zones optimal?
% Should you count the total number of objects and decrease count with each
% pick to ensure you have cleared an area? Should you visualize a scene
% after each pick attempt?
%
% TODO: pick function can be optimized for new objects like spam/markers
% (only to pursue if you train the Yolo Network to identify these.
%--------------------------------------------------------------------------
    
    % Inspect the zone and get the number of subZones and joint configurations for each subZone
    [startZone] = returnZoneJointConfig(zoneInspect);
    
    % Move the robot to the inspected object's joint configuration
    cprintf('blue', '%s: ', zoneInspect); fprintf('01 Moving to zone at config %s...\n', strjoin(string(startZone)) );
    optns{'traj_duration'} = .1;
    moveToQ("Custom",optns,startZone);
    
    % Capture an image of the zone and detect objects
    cprintf('blue', '%s: ', zoneInspect); fprintf('Object Detection...\n');
    if exist(string(zoneInspect)+"-objectsData.mat", "file")
        saveFile = load(string(zoneInspect)+"-objectsData.mat");
        objects = saveFile.objects;
        W_T_ptCloud = saveFile.W_T_ptCloud;
        W_T_R = saveFile.W_T_R;
        R_T_C = saveFile.R_T_C;
        frustums = saveFile.frustums;
    else
        [objects, W_T_ptCloud, ~, ~, W_T_R, R_T_C, frustums] = locateObjectsHere(optns);
        save(string(zoneInspect)+"-objectsData.mat", 'objects', 'W_T_ptCloud', 'W_T_R', 'R_T_C', 'frustums');
    end

    debugShowZone(objects, W_T_ptCloud, W_T_R, R_T_C, frustums)
    
    % Iterate over each detected object
    numObjects = size(objects,1);
    cprintf('blue', '%s: ', zoneInspect); fprintf('Identified %d objects...\n', numObjects);
    for j = 1:numObjects
        obj = objects{j};
        cprintf('magenta', ' - Object %d: ', j); cprintf('text', 'Picking %s @ (%.2f, %.2f, %.2f)...\n', obj.label, obj.topCenterPosition(1), obj.topCenterPosition(2), obj.topCenterPosition(3));
        % pick
        fprintf('    - Picking...\n');
        moveToQ("Custom", optns, startZone);
        pick("topdown", obj.pickPose, obj.label, optns);
        
        % place
        fprintf('    - Placing...\n');
        moveToQ("Custom", optns, startZone);
        place("topdown", obj.label, optns); % label for knowing which bin to go to
    
    end
    pause(5);

end
