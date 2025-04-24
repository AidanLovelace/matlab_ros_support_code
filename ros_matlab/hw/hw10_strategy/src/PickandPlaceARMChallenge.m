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

    logPrint(4, 'PickandPlaceARMChallenge-'+string(zoneInspect), 0, 'blue', "Retreived zone start configuration: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", startZone(1), startZone(2), startZone(3), startZone(4), startZone(5), startZone(6));

    % Move the robot to the inspected object's joint configuration
    logPrint(1, 'PickandPlaceARMChallenge-'+string(zoneInspect), 0, '', "Moving to zone start position");
    moveToQ("Custom",optns,startZone);

    % Capture an image of the zone and detect objects
    logPrint(0, 'PickandPlaceARMChallenge-'+string(zoneInspect), 0, 'blue', "Object Detection");
    if optns{'useZoneObjectsCache'} == 1
        logPrint(1, 'PickandPlaceARMChallenge-'+string(zoneInspect), 1, '', "Zone Objects Cache is enabled. Attemping to load from cache.");
        if exist(string(zoneInspect)+"-objectsData.mat", "file")
            saveFile = load(string(zoneInspect)+"-objectsData.mat");
            objects = saveFile.objects;
            W_T_ptCloud = saveFile.W_T_ptCloud;
            W_T_R = saveFile.W_T_R;
            R_T_C = saveFile.R_T_C;
            frustums = saveFile.frustums;
            logPrint(1, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Objects loaded successfully from cache.");
        else
            logPrint(1, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "No cache found. Will perform object detection as normal.");
        end
    end

    if ~exist('objects', 'var')
        [objects, W_T_ptCloud, ~, ~, W_T_R, R_T_C, frustums] = locateObjectsHere(optns, zoneInspect);
        if optns{'useZoneObjectsCache'} == 1
            % save(string(zoneInspect)+"-objectsData.mat", 'objects', 'W_T_ptCloud', 'W_T_R', 'R_T_C', 'frustums');
            logPrint(1, 'PickandPlaceARMChallenge-'+string(zoneInspect), 1, '', "Zone Objects Cache saved.");
        end
    end

    logPrint(1, 'PickandPlaceARMChallenge-'+string(zoneInspect), 1, '', "Showing debug visualization of this zone, including the point cloud, frustums, bounding boxes, and the fitted object models.");
    % debugShowZone(objects, W_T_ptCloud, W_T_R, R_T_C, frustums)

    % Iterate over each detected object
    numObjects = size(objects,1);
    logPrint(1, 'PickandPlaceARMChallenge-'+string(zoneInspect), 1, 'blue', "Picking the Objects.");
    for j = 1:numObjects
        obj = objects{j};
        logPrint(1, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, 'magenta', "Object %d: %s @ (%.2f, %.2f, %.2f)", j, obj.label, obj.topCenterPosition(1), obj.topCenterPosition(2), obj.topCenterPosition(3));

        logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Moving to zone start position.");
        moveToQ("Custom", optns, startZone);
        try
            if strcmpi(obj.label, 'vBottle')
                logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Knocking over the bottle. Will come back to pick it up.");
                % Knock over the bottle and move on.
                knockPos1 = obj.pickPose;
                knockPos2 = obj.pickPose;
                knockPos3 = obj.pickPose;
                knockPos1(2,4) = knockPos1(2,4) - 0.05;
                knockPos1(1,4) = knockPos1(1,4) - 0.04;
                knockPos1(3,4) = knockPos1(3,4) + 0.15;
                knockPos2(2,4) = knockPos2(2,4) - 0.05;
                knockPos2(1,4) = knockPos2(1,4) - 0.04;
                knockPos3(2,4) = knockPos3(2,4) + 0.02;
                doGrip("pick", optns, 1);
                moveTo(knockPos1, optns);
                pause(0.3);
                moveTo(knockPos2, optns);
                pause(0.3);
                moveTo(knockPos3, optns);
                doGrip("pick", optns, 0);
            else
                logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Picking object up.");
                pick("topdown", obj.pickPose, obj.label, optns);
                logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Moving back to zone start position.");
                moveToQ("Custom", optns, startZone);
                if optns{'comparePicksWithGazebo'}
                    pause(0.3);
                    % Check if we actually picked the object
                    [currentGripperPos, currentModelPos] = get_robot_object_pose_wrt_base_link(obj.gazeboMatch.modelName, 1, optns);
                    currentModelPos = [currentModelPos(1, 4), currentModelPos(2, 4), currentModelPos(3, 4)];
                    currentGripperPos = [currentGripperPos(1, 4), currentGripperPos(2, 4), currentGripperPos(3, 4)];
                    d = sqrt(sum((currentModelPos - currentGripperPos) .^ 2));
                    if abs(d - obj.gazeboMatch.dist) > 0.2
                        logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '*red', "According to model position in Gazebo, failed to pick object. Gripper-to-'%s' Distance: %.2f", obj.gazeboMatch.modelName, d);
                        logPrint(3, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Gripper Pos: [%.2f, %.2f, %.2f], Model Pos: [%.2f, %.2f, %.2f]", currentGripperPos(1), currentGripperPos(2), currentGripperPos(3), currentModelPos(1), currentModelPos(2), currentModelPos(3));
                    else
                        logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '*green', "According to model position in Gazebo, successfully picked object. Gripper-to-'%s' Distance: %.2f", obj.gazeboMatch.modelName, d);
                        logPrint(3, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Gripper Pos: [%.2f, %.2f, %.2f], Model Pos: [%.2f, %.2f, %.2f]", currentGripperPos(1), currentGripperPos(2), currentGripperPos(3), currentModelPos(1), currentModelPos(2), currentModelPos(3));
                    end
                end
                logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Placing object in bin.");
                place("topdown", obj.label, optns); % label for knowing which bin to go to
                pause(0.3);
                doGrip("pick", optns, 1);
                pause(0.3);
                doGrip("pick", optns, 0);
            end
        catch
            logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '*red', "Failed to pick Object %d: %s @ (%.2f, %.2f, %.2f)", j, obj.label, obj.topCenterPosition(1), obj.topCenterPosition(2), obj.topCenterPosition(3));
        end
    end
    pause(5);

end
