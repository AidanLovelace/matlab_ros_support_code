function [objects, W_T_ptCloud, myImg, annotatedImage, W_T_R, R_T_C, frustums] = locateObjectsHere(optns, zoneInspect)
    r = optns{'rHandle'};
    tftree = rostf('DataFormat','struct');
    % [startZone] = get_current_joint_states(optns);

    % Take a picture
    rosImg  = receive(r.rgb_sub);
    myImg   = rosReadImage(rosImg,"PreserveStructureOnRead",true);
    logPrint(1, 'locateObjectsHere-'+string(zoneInspect), 1, '', "Picture taken.");
    % Dispatch processing the image 
    futureImage = parfeval(@getLabeledImg, 5, myImg, r.general_detector);
    logPrint(2, 'locateObjectsHere-'+string(zoneInspect), 1, '', "YOLO detection on image started in 2nd thread.");
    
    % Get the point cloud
    logPrint(1, 'locateObjectsHere-'+string(zoneInspect), 1, '', "Start merged point cloud retrieval.");
    [~, R_T_ptCloud, ~] = getMergedPTC(optns, "all");
    logPrint(1, 'locateObjectsHere-'+string(zoneInspect), 1, '', "Merged point cloud retrieved.");
    
    % Get transforms to points of interest
    W_T_R = get_model_pose('robot', optns);
    R_T_C = getTransform(tftree, 'base_link', 'camera_depth_link', rostime('now'),'Timeout', r.tf_listening_time);
    
    % Transform point cloud to world coords
    W_T_ptCloud = pctransform(R_T_ptCloud,rigidtform3d(ros2MLPose(W_T_R)));
    
    % Get the camera info
    logPrint(2, 'locateObjectsHere-'+string(zoneInspect), 1, '', "Matching bounding boxes from YOLO model to point cloud to get point clouds of individual objects.");
    camInfo = receive(r.caminfo_sub);
    logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 2, '', "Received camera info (including intrinsics) from camera.");
    camPose = ros2MLPose(W_T_R) * ros2MLPose(R_T_C);
    logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 2, '', "Waiting for the YOLO detection to finish.");
    wait(futureImage);
    [bboxes, ~, labels, numOfObjects, annotatedImage] = fetchOutputs(futureImage);
    logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 2, '', "YOLO detection finished.");
    logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 3, '', "Number Objects: %d", numOfObjects);
    labelsStr = sprintf(', "%s"', labels);
    logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 3, '', "Labels: [%s]", labelsStr(2:end));
    frustumCornersWorld = computeFrustumFromBBox(camInfo, rigidtform3d(camPose), bboxes, 0.1, 1);
    frustums = frustumCornersWorld;
    logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 2, '', "Frustums of bounding boxes computed.");

    logPrint(2, 'locateObjectsHere-'+string(zoneInspect), 2, '', "Processing each object.");
    objects = cell(numOfObjects,1);
    for i = 1:numOfObjects
        logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 3, 'magenta', "Object %d: %s", i, labels(i));
        % Filter point cloud to only include points within the frustum of this bounding box
        objectPtCloud = selectPointsInFrustum(W_T_ptCloud, frustumCornersWorld(:,:,i));
        label = string(labels(i));
        logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 4, '', "Filtered Point Cloud, %d points.", size(objectPtCloud.Location, 1));
        logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 4, '', "Attempting to fit a pick pose.");
        [model, topCenterPosition, pickPose] = fitPickPose(objectPtCloud, label);
        
        if isempty(model)
            logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 4, 'red', "Pick pose fit failed. This object will not be picked.");
            continue;
        end

        if optns{'comparePicksWithGazebo'}
            [gazeboModelName, gazeboModelPos, gazeboModelDist] = findNearestGazeboModel(topCenterPosition, optns);
            if gazeboModelDist < 0.1
                logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 4, '*green', "Matched object to gazebo model. Model Name: %s, Model Position: (%.2f, %.2f, %.2f), Distance: %.2f", gazeboModelName, gazeboModelPos(1), gazeboModelPos(2), gazeboModelPos(3), gazeboModelDist);
            else
                logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 4, '*red', "Failed to match object to gazebo model.");
            end
        end
        
        W_T_R2 = ros2MLRobotPose(get_model_pose('robot', optns), 1, 1, optns);
        R_T_B = W_T_R2\pickPose;
        R_T_B(3,4) = R_T_B(3,4) - 0.06;
        
        logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 4, '', "Pick pose offset by 6cm down so the gripper is actually around the object.");
        
        % Comparing z coordinates of each end of the cylinder to determine if it is vertical or horizontal.
        orientation = "unknown";
        if (abs(model.Parameters(6) - model.Parameters(3)) > 0.04)
            % This object is likely vertical.
            orientation = "vertical";
        else
            % This object is likely horizontal.
            orientation = "horizontal";
        end
        % Override the above orientation if the bounding box is far from being a square.
        if (bboxes(i, 3)/bboxes(i, 4) > 1.5) || (bboxes(i, 3)/bboxes(i, 4) < 0.75)
            % This object is likely vertical.
            orientation = "horizontal";
        end

        logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 4, '', "This object is likely %s.", orientation);
        
        label = char(label);
        if strcmp(label, "can") || strcmp(label, "bottle")
            if strcmp(orientation, "horizontal")
                label(1)=upper(label(1)); % capitalize the first letter
                label = "h" + label;
            elseif strcmp(orientation, "vertical")
                label(1)=upper(label(1)); % capitalize the first letter
                label = "v" + label;
                if contains(label, 'ottle')
                    R_T_B = R_T_B * trotx(-3*pi/8);
                    R_T_B(2,4) = R_T_B(2,4) + 0.13;
                    R_T_B(3,4) = R_T_B(3,4) - 0.1;
                end
            end
            logPrint(4, 'locateObjectsHere-'+string(zoneInspect), 4, '', "Object relabeled to %s.", label);
        end
        
        objectData.ptCloud = objectPtCloud;
        objectData.label = label;
        objectData.bbox = bboxes(i,:);
        objectData.frustum = frustumCornersWorld(:,:,i);
        objectData.model = model;
        objectData.topCenterPosition = topCenterPosition;
        objectData.pickPose = R_T_B;
        if optns{'comparePicksWithGazebo'}
            objectData.gazeboMatch = struct();
            objectData.gazeboMatch.modelName = gazeboModelName;
            objectData.gazeboMatch.modelPos = gazeboModelPos;
            objectData.gazeboMatch.dist = gazeboModelDist;
        end
        objects{i} = objectData;
    end
    objects = objects(~cellfun('isempty',objects));
    logPrint(2, 'locateObjectsHere-'+string(zoneInspect), 3, '', "%d objects will be picked. %d skipped.", size(objects,1), numOfObjects - size(objects,1));
end