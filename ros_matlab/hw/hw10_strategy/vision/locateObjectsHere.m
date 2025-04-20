function [objects, W_T_ptCloud, myImg, annotatedImage, W_T_R, R_T_C, frustums] = locateObjectsHere(optns)
    r = optns{'rHandle'};
    tftree = rostf('DataFormat','struct');
    % [startZone] = get_current_joint_states(optns);

    % Take a picture
    rosImg  = receive(r.rgb_sub);
    myImg   = rosReadImage(rosImg,"PreserveStructureOnRead",true);
    % Dispatch processing the image 
    futureImage = parfeval(@getLabeledImg, 5, myImg, r.general_detector);

    % Get the point cloud
    [~, R_T_ptCloud, ~] = getMergedPTC(optns, "all");
    
    % Get transforms to points of interest
    W_T_R = get_model_pose('robot', optns);
    R_T_C = getTransform(tftree, 'base_link', 'camera_depth_link', rostime('now'),'Timeout', r.tf_listening_time);

    % Transform point cloud to world coords
    W_T_ptCloud = pctransform(R_T_ptCloud,rigidtform3d(ros2MLPose(W_T_R)));

    % Get the camera info
    camInfo = receive(r.caminfo_sub);
    camPose = ros2MLPose(W_T_R) * ros2MLPose(R_T_C);
    wait(futureImage);
    [bboxes, ~, labels, numOfObjects, annotatedImage] = fetchOutputs(futureImage);
    frustumCornersWorld = computeFrustumFromBBox(camInfo, rigidtform3d(camPose), bboxes, 0.1, 1);
    frustums = frustumCornersWorld;

    objects = cell(numOfObjects,1);
    for i = 1:numOfObjects
        % Filter point cloud to only include points within the frustum of this bounding box
        objectPtCloud = selectPointsInFrustum(W_T_ptCloud, frustumCornersWorld(:,:,i));
        label = string(labels(i));
        [model, topCenterPosition, pickPose] = fitPickPose(objectPtCloud, label);

        if isempty(model)
            continue;
        end

        W_T_R2 = ros2MLRobotPose(get_model_pose('robot', optns), 1, 1, optns);
        R_T_B = W_T_R2\pickPose;
        R_T_B(3,4) = R_T_B(3,4) - 0.06; % Offset along +z_base_link to simulate knowing height of top of can.

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

        label = char(label);
        if strcmp(label, "can") || strcmp(label, "bottle")
            if strcmp(orientation, "horizontal")
                label(1)=upper(label(1)); % capitalize the first letter
                label = "h" + label;
            elseif strcmp(orientation, "vertical")
                label(1)=upper(label(1)); % capitalize the first letter
                label = "v" + label;
            end
        end
        
        objectData.ptCloud = objectPtCloud;
        objectData.label = label;
        objectData.bbox = bboxes(i,:);
        objectData.frustum = frustumCornersWorld(:,:,i);
        objectData.model = model;
        objectData.topCenterPosition = topCenterPosition;
        objectData.pickPose = R_T_B;
        objects{i} = objectData;
    end
    objects = objects(~cellfun('isempty',objects))
end