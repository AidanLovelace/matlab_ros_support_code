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