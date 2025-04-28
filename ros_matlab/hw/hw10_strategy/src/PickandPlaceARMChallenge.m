function [zoneComplete] = PickandPlaceARMChallenge(zoneInspect, robotHandle, options)
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
    objectIdentifier = robotHandle.objectIdentifier;

    if startsWith(zoneInspect, 'static-')
        if strcmp(zoneInspect, 'static-Zone1')
            objects = load("./staticZone1.mat").objects;
        elseif strcmp(zoneInspect, 'static-Zone2')
            objects = load("./staticZone2.mat").objects;
        end

        for j = 1:length(objects)
            obj = objects{j};
            logPrint(1, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, 'magenta', "Object %d: %s", j, obj.label);

            logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Moving to zone start position.");
            robotHandle.moveToStdPosition(zoneInspect, 0.25);

            try
                logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Picking object up.");
                robotHandle.pick(obj);
                logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Moving back to zone start position.");
                % robotHandle.moveToStdPosition(zoneInspect, 0.25);

                logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Placing object in bin.");
                robotHandle.place(obj); % label for knowing which bin to go to

                pause(1);
            catch
                logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '*red', "Failed to pick Object %d: %s", j, obj.label);

            end
        end
    else
        % Move the robot to the inspected object's joint configuration
        logPrint(1, 'PickandPlaceARMChallenge-'+string(zoneInspect), 0, '', "Moving to zone start position");
        robotHandle.moveToStdPosition(zoneInspect, 0.25);
        robotHandle.moveToStdPosition(zoneInspect, 0.25);
        pause(0.5);

        % Capture an image of the zone and detect objects
        logPrint(0, 'PickandPlaceARMChallenge-'+string(zoneInspect), 0, 'blue', "Object Detection");

        [objects, ~] = objectIdentifier.identifyObjectsHere(zoneInspect);

        % logPrint(1, 'PickandPlaceARMChallenge-'+string(zoneInspect), 1, '', "Showing debug visualization of this zone, including the point cloud, frustums, bounding boxes, and the fitted object models.");
        % debugShowZone(objects, W_T_ptCloud, W_T_R, R_T_C, frustums)

        % Iterate over each detected object
        logPrint(1, 'PickandPlaceARMChallenge-'+string(zoneInspect), 1, 'blue', "Picking the Objects.");

        for j = 1:size(objects,1)

            obj = objects{j};

            logPrint(1, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, 'magenta', "Object %d: %s @ (%.2f, %.2f, %.2f)", j, obj.label, obj.topCenterPosition(1), obj.topCenterPosition(2), obj.topCenterPosition(3));

            logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Moving to zone start position.");
            robotHandle.moveToStdPosition(zoneInspect, 0.25);

            try
                % if strcmpi(obj.label, 'vBottle')
                %     logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Knocking over the bottle. Will come back to pick it up.");

                %     % Knock over the bottle and move on.
                %     knockPos1 = obj.pickPose;
                %     knockPos2 = obj.pickPose;
                %     knockPos3 = obj.pickPose;
                %     knockPos1(2,4) = knockPos1(2,4) - 0.05;
                %     knockPos1(1,4) = knockPos1(1,4) - 0.04;
                %     knockPos1(3,4) = knockPos1(3,4) + 0.15;
                %     knockPos2(2,4) = knockPos2(2,4) - 0.05;
                %     knockPos2(1,4) = knockPos2(1,4) - 0.04;
                %     knockPos3(2,4) = knockPos3(2,4) + 0.02;
                %     robotHandle.setGripper(1);
                %     robotHandle.moveToPose(knockPos1);
                %     pause(0.3);
                %     robotHandle.moveToPose(knockPos2);
                %     pause(0.3);
                %     robotHandle.moveToPose(knockPos3);
                %     robotHandle.setGripper(0);
                % else
                    logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Picking object up.");
                    robotHandle.pick(obj);
                    logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Moving back to zone start position.");
                    % robotHandle.moveToStdPosition(zoneInspect, 0.25);

                    logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '', "Placing object in bin.");
                    robotHandle.place(obj); % label for knowing which bin to go to
                % end
            catch
                logPrint(2, 'PickandPlaceARMChallenge-'+string(zoneInspect), 2, '*red', "Failed to pick Object %d: %s @ (%.2f, %.2f, %.2f)", j, obj.label, obj.topCenterPosition(1), obj.topCenterPosition(2), obj.topCenterPosition(3));

            end
        end

        pause(1);
        robotHandle.moveToStdPosition(zoneInspect, 0.25);
        pause(1);
    end
end
