function testPtCloudImgBBoxOverlap(optns)
    currentFolderContents = dir(pwd);      % Returns all files and folders in the current folder
    currentFolderContents (~[currentFolderContents.isdir]) = [];  % Only keep the folders

    for i = 3:length(currentFolderContents) % Start with 3 to avoid '.' and '..' 
        addpath(['./' currentFolderContents(i).name]) 
    end

    [startZone] = returnZoneJointConfig("Zone1");
    goHome("qr", optns);
    moveToQ("Custom",optns,startZone);
    resetWorld(optns);

    [objects, W_T_ptCloud, ~, ~, W_T_R, R_T_C, frustums] = locateObjectsHere(optns);
    
    debugShowZone(objects, W_T_ptCloud, W_T_R, R_T_C, frustums);

    for i = 1:size(objects,1)
        obj = objects{i};
        pick("topdown", obj.pickPose,optns);
        moveToQ("Custom",optns, startZone);
        place("topdown", "can", optns); % label for knowing which bin to go to
        moveToQ("Custom", optns, startZone);
    end
end

function debugShowZone(objects, W_T_ptCloud, W_T_R, R_T_C, frustums)
    figure;
    pcshow(W_T_ptCloud);
    hold on;
    for i = 1:size(objects,1)
        obj = objects{i};
        cprintf('text', ' - %s - (%.2f, %.2f, %.2f)...\n', obj.label, obj.topCenterPosition(1), obj.topCenterPosition(2), obj.topCenterPosition(3));
        % Plot the fitted cylinder
        plot(obj.model);
        hold on;
        % Plot the top center point of the cylinder
        scatter3(obj.topCenterPosition(1), obj.topCenterPosition(2), obj.topCenterPosition(3), 150, [1 0 1], 'filled');
        text(obj.topCenterPosition(1), obj.topCenterPosition(2), obj.topCenterPosition(3), obj.label, "Color", [1 1 1], "FontWeight", "bold", "HorizontalAlignment", "right", "VerticalAlignment", "bottom");
        hold on;
    end

    % Get positions of points of interest
    wtr = [W_T_R.Pose.Position.X;W_T_R.Pose.Position.Y;W_T_R.Pose.Position.Z];
    rtc = [R_T_C.Transform.Translation.X;R_T_C.Transform.Translation.Y;R_T_C.Transform.Translation.Z];
    wtc = wtr + rtc;

    u = [wtr(1) wtc(1)];
    v = [wtr(2) wtc(2)];
    Z = [wtr(3) wtc(3)];

    axis on;
    xlabel("X"); ylabel("Y"); zlabel("Z");
    hold on;
    scatter3(u, v, Z, 100, [1 0 0; 1 0 0], 'filled');
    text(u, v, Z, ["Robot Base", "Camera"], "Color", [1 1 1], "FontWeight", "bold", "HorizontalAlignment", "center", "VerticalAlignment", "bottom");
    hold on;
    visualizeFrustum(frustums);
    title('Debug Display: Whole Zone'); hold off;
end

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
    [~, R_T_ptCloud, ~] = getMergedPTC(optns, {});
    
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

        % if model.Radius > 0.1
        %     % skip this one
        %     continue;
        % end

        W_T_R2 = ros2MLRobotPose(get_model_pose('robot', optns), 1, 1, optns);
        R_T_B = W_T_R2\pickPose;
        R_T_B(3,4) = R_T_B(3,4) - 0.06; % Offset along +z_base_link to simulate knowing height of top of can.

        objectData.ptCloud = objectPtCloud;
        objectData.label = label;
        objectData.bbox = bboxes(i,:);
        objectData.frustum = frustumCornersWorld(:,:,i);
        objectData.model = model;
        objectData.topCenterPosition = topCenterPosition;
        objectData.pickPose = R_T_B;
        objects{i} = objectData;
    end
end

function frustumCornersWorld = computeFrustumFromBBox(camInfo, camPose, bboxes, zNear, zFar)
    % computeFrustumFromBBox Compute a 3D frustum in world coordinates for a 2D image bbox
    %   frustumCornersWorld = computeFrustumFromBBox(camInfo, camPose, bbox, zNear, zFar)
    % Inputs:
    %   camInfo - ROS sensor_msgs/CameraInfo struct
    %   camPose - rigid3d camera-to-world pose
    %   bbox    - Nx4 - [x, y, width, height] in pixels
    %   zNear   - scalar near-plane distance (in meters)
    %   zFar    - scalar far-plane distance (in meters)
    %
    % Output:
    %   frustumCornersWorld - 8x3xN array of world-frame points: [nearTL; nearTR; nearBR; nearBL; farTL; farTR; farBR; farBL]
    
    frustumCornersWorld = zeros(8,3,size(bboxes,1));

    for i = 1:size(bboxes,1)
        bbox = bboxes(i,:);
        % Build intrinsic matrix
        K = reshape(camInfo.K, 3, 3)';
        fx = K(1,1); fy = K(2,2);
        cx = K(1,3); cy = K(2,3);
        
        % Pixel coordinates of bbox corners
        u = [bbox(1),       bbox(1) + bbox(3), bbox(1) + bbox(3), bbox(1)];
        v = [bbox(2),       bbox(2),           bbox(2) + bbox(4), bbox(2) + bbox(4)];
        
        % Convert pixels to normalized camera directions
        dirs = [(u(:) - cx) / fx, (v(:) - cy) / fy, ones(4,1)];
        % Normalize directions (optional, for consistent scaling)
        dirs = dirs ./ vecnorm(dirs,2,2);
        
        % Compute 3D points in camera frame at near and far planes
        ptsNearCam = dirs * zNear;
        ptsFarCam  = dirs * zFar;
        
        % Combine into 8 corners
        ptsCam = [ptsNearCam; ptsFarCam];  % 8x3
        
        % Transform to world frame using camPose
        frustumCornersWorld(:,:,i) = transformPointsForward(camPose, ptsCam);
    end

    
end
    
    
function visualizeFrustum(frustumCornersWorld)
    % visualizeFrustum Display a 3D frustum given its world corners
    %   visualizeFrustum(frustumCornersWorld)
    % Input:
    %   frustumCornersWorld - 8x3xN array as returned by computeFrustumFromBBox
    %
    % The order of rows is: [nearTL; nearTR; nearBR; nearBL; farTL; farTR; farBR; farBL]
    
    for i = 1:size(frustumCornersWorld,3)
        % Split into near and far planes
        nearPts = frustumCornersWorld(1:4, :, i);
        farPts  = frustumCornersWorld(5:8, :, i);
        
        % Draw edges of near and far rectangles
        for j = 1:4
            ni = nearPts(j, :);
            fi = farPts(j, :);
            nj = nearPts(mod(j,4)+1, :);
            fj = farPts(mod(j,4)+1, :);
            % Near edge
            plot3([ni(1), nj(1)], [ni(2), nj(2)], [ni(3), nj(3)], 'r-');
            % Far edge
            plot3([fi(1), fj(1)], [fi(2), fj(2)], [fi(3), fj(3)], 'r-');
            % Side edges
            plot3([ni(1), fi(1)], [ni(2), fi(2)], [ni(3), fi(3)], 'r-');
        end
        hold on;
    end
end


function subsetCloud = selectPointsInFrustum(ptCloud, frustumCornersWorld)
    % selectPointsInFrustum Extract points of a ptCloud within the specified frustum
    %   subsetCloud = selectPointsInFrustum(ptCloud, frustumCornersWorld)
    % Inputs:
    %   ptCloud               - pointCloud object in world coordinates
    %   frustumCornersWorld   - 8x3 frustum corners as [nearTL; nearTR; nearBR; nearBL; farTL; farTR; farBR; farBL]
    %
    % Output:
    %   subsetCloud           - pointCloud containing only points inside the frustum
    
    % Define faces of frustum (each row indices into corners)
    faces = [1 2 3; 1 3 4; 5 6 7; 5 7 8; 1 2 6; 2 3 7; 3 4 8; 4 1 5];
    corners = frustumCornersWorld;
    
    % Compute centroid to orient normals inward
    centroid = mean(corners,1);
    
    % Preallocate plane coefficients (8 planes, 4 coeffs each)
    numFaces = size(faces,1);
    planes = zeros(numFaces,4);
    
    % Build each plane: [n_x n_y n_z d]
    for f = 1:numFaces
        idx = faces(f,:);
        p1 = corners(idx(1),:);
        p2 = corners(idx(2),:);
        p3 = corners(idx(3),:);
        n = cross(p2-p1, p3-p1);    % 1x3 row vector
        n = n / norm(n);
        d = -dot(n,p1);
        % flip to point inward
        if dot(n,centroid) + d < 0
            n = -n; d = -d;
        end
        planes(f,:) = [n d];    % use row vector n
    end
    
    % Flatten point cloud locations
    locs = ptCloud.Location;
    flat = reshape(locs, [], 3);
    
    % Test all points against all half-spaces
    maskAll = true(size(flat,1),1);
    for f = 1:numFaces
        n = planes(f,1:3);
        d = planes(f,4);
        maskAll = maskAll & (flat * n' + d >= 0);
    end
    
    % Select and return subset via built-in select
    indices = find(maskAll);
    subsetCloud = select(ptCloud, indices);
end

% Start with only cylinders
function [model, topCenterPosition, pickPose] = fitPickPose(objectPtCloud, label)
    % Assume Cylinder and that it's either upright or lying on its side. Nowhere in between.
    
    % Attempt to fit an upright cylinder to the filtered point cloud
    [model, inIndices, ~, ~] = pcfitcylinder(objectPtCloud, 0.005, [0 0 1]);
    if inIndices < 0.99*length(objectPtCloud.Location)
        [model, inIndices, ~, ~] = pcfitcylinder(objectPtCloud, 0.005);
    end

    if model.Radius > 0.1
        % If the radius is too large, we assume it's not a cylinder
        % and return an empty model
        model = [];
        topCenterPosition = [];
        return;
    end

    % Find the best pick pose by finding whether the cylinder is upright or lying on its side
    % and then determining the rotation of the gripper to align with the cylinder

    % First, find the angle between the cylinder's axis and the z-axis
    pickPose = [];
    p1 = model.Parameters(1:3);
    p2 = model.Parameters(4:6);
    vertAngle = acosd( (p2(3)-p1(3)) / norm(p2 - p1) );
    if vertAngle < 45
        pickPose = transl(-model.Center(2), model.Center(1), objectPtCloud.ZLimits(1,2)) * trotx(-pi);
    else
        % FIXME: This is a placeholder
        pickPose = transl(-model.Center(2), model.Center(1), objectPtCloud.ZLimits(1,2)) * trotx(-pi) * trotz(pi/2);
    end

    % Get the top center of the cylinder
    topCenterPosition = [model.Center(1), model.Center(2), objectPtCloud.ZLimits(1,2)];
end