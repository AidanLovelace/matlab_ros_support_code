function objectData = getObjectData(ptCloud_base, nonPlane_pic, myImg, bboxes, numOfObjects, base_to_cam_pose, cam_to_base_pose, labeled)    
%--------------------------------------------------------------------------
% Transforms point clouds from the camera frame to the robot's base frame,
% estimates object poses using PCA and downsampling, visualizes each
% object's point cloud with its principal axes, constructs object pose
% transformations, cleans the results to remove incomplete data, and
% refines object labels based on orientation.
%
% Main Steps:
% 01 Transform point clouds from cam to base as reference
% 02 Compute xyz theta pose information
% 03 Visualize pt clouds for objects + pose
% 04 Compute 4x4 Poses from Merged Point Clouds
% 05 Consider objects that do not have NaNs
% 06 Refine Classification
%
%   Inputs:
%       ptCloud_base    - The primary point cloud in the base frame.
%       nonPlane_pic    - The point cloud representing non-planar features.
%       myImg           - Corresponding RGB image for context.
%       bboxes          - Bounding boxes (e.g., from object detection like YOLO).
%       numOfObjects    - Number of objects detected in the scene.
%       base_to_cam_pose- Transformation matrix from base frame to camera frame.
%       cam_to_base_pose- Transformation matrix from camera frame to base frame.
%       labeled         - Array of object labels.
%
%   Outputs:
%       objectData      - A cell array where each row contains {label, 4x4 object pose, 
%                         downsampled point cloud} for an object.
%
%   Example:
%       objectData = getObjectData(ptCloud_base, nonPlane_pic, myImg, bboxes, numOfObjects, 
%                                  base_to_cam_pose, cam_to_base_pose, labeled);
%
%   See also: pctransform, rigidtform3d, betterObjectPoses, pcshow.
%
%--------------------------------------------------------------------------
    %% 01 Transform all point cloud points to base_to_cam 

    % Convert cam_to_base_pose to a matlab rigitform3d similar to homTrans in Peter Corke 
    tform_to_cam = rigidtform3d(cam_to_base_pose);

    % TODO: Transform the reference frame of point clouds ptCloud_base with tform_to_cam
    ptCloud_tform_cam = 

    % TODO: Also do for non-plane specific points nonPlane_pic with tform_to_cam
    nonPlane_tform_cam = 
    
    %% 02 Estimate object pose wrt to base
    disp("Finding object pose with respect to base_link...")

    % Downsample number of points clouds to minimize compute, reduce noise/outliers
    gridDownsample = 0.007;
    
    % If using a single point cloud:
    %[xyz,theta,ptCloud_vec,scene_pca_vec] = findObjectPoses(ptCloud_tform, myImg, bboxes, gridDownsample, nonPlaneMask);

    % If using merged point clouds:
    [xyz,theta,ptCloud_vec,scene_pca_vec] = betterObjectPoses(ptCloud_tform_cam, ...    % all points wrt to base
                                                              nonPlane_tform_cam, ...   % non plane points wrt to base
                                                              myImg, ...                % rgb image
                                                              bboxes, ...               % yolo bounding boxes
                                                              gridDownsample, ...       % downsample factor
                                                              base_to_cam_pose);        % base to cam pose
    
    %% 03 Visualize Objects     
    
    figure;
    for idx = 1: numOfObjects

        %% FIXME: convert the following code to a function visualize_xyzth(scene_pca_vec)
        
        % Extract the principal axes (UVW) from the PCA result for the current object.
        % The UVW matrix is expected to be a 3-by-3 matrix where: UVW are the 1st/2nd/3rd principal components
        U = scene_pca_vec{idx}.UVW(:,1);    % U-axis: 1st component (maximal variance)
        V = scene_pca_vec{idx}.UVW(:,2);    % V-axis: 2nd component
        W = scene_pca_vec{idx}.UVW(:,3);    % W-axis: 3rd component (minimal variance)

        % Retrieve ptcloud centroid (used to draw principal axes).
        center = scene_pca_vec{idx}.centroid;

        % Create a new subplot tile for this object's visualization.
        nexttile;

        % Render point cloud ptCloud_vec.
        pcshow(ptCloud_vec{idx},'ViewPlane','XY');
        xlabel("X"); ylabel("Y"); zlabel("Z"); title(idx); hold on;

        % Draw xyz vectors from centroid on out in RGB colors
        quiver3(center(1), center(2), center(3), U(1), V(1), W(1), 'r');
        quiver3(center(1), center(2), center(3), U(2), V(2), W(2), 'g');
        quiver3(center(1), center(2), center(3), U(3), V(3), W(3), 'b');
        hold off;
    end
    
    %% 04 Compute Pose for each object (4x4) from xyz theta

    % Intialize a vector of cells
    objPoses = cell(1,numOfObjects);
    
    % For each object
    for i = 1: numOfObjects

        %% Extract position
        position = [xyz(i,1); xyz(i,2); xyz(i,3)];

        %% Extract yaw in radians
        angle = deg2rad(theta(i));

        % **Orientation Adjustments. 
        % We want the gripper to do a top-down pick but rotated by yaw
        rotation = eul2tform([-pi/2,pi,0])*trotz(angle);

        %% Matlab Adjustments! Given that matlab modeled the robot backwards, 
        % ... We need to adjust the orientation of the rotation matrix. 
        % TODO: We have a function already for this need to integrate
        col2 = rotation(:,2);
        col1 = rotation(:,1);

        rotation(:,1) = col2;
        rotation(:,2) = -col1;

        % Adjust ros2mat positions
        rotation(1,4) = -position(2);
        rotation(2,4) = position(1);
        rotation(3,4) = position(3);

        % Copy to obPoses array entry
        objPoses{i} = rotation;
    end


    %% 05 Clean Output
    % Only keep point clouds and data for which there is no NaN. This
    % allows us to work with point clouds whose data is complete and
    % dimensions unchanged.
    
    % Output container will hold labels, poses, and pt_clouds
    objectData = cell(0, 3);

    % Clean data for each object 
    for j = 1:numOfObjects
        
        % Check for NaN values; if so skip and do not copy out.
        if any(isnan(objPoses{j}), 'all')
            continue; 
        end
        
        % Add the label and pose to objectData
        objectData(end+1, :) = {char(labeled(j)), ...   % Label
                                objPoses{j},...         % Pose
                                ptCloud_vec{j}};        % Pt Cloud
    end
   
    %% 06 ReFine Classification - Modify labels to understand if we have vertical or horizontal bottles/cans.
    for k = 1:size(objectData,1)
        if (objectData{k,3}.ZLimits(2) > 0) && (objectData{k,1} == "bottle" )
            objectData{k,1} = 'vBottle';
        
        elseif (objectData{k,3}.ZLimits(2) < 0) && (objectData{k,1} == "bottle" )
            objectData{k,1} = 'hBottle';
        
        elseif (objectData{k,3}.ZLimits(2) > 0) && (objectData{k,1} == "can" ) %%% needs to be tested Zlimits might not look right
            objectData{k,1} = 'vCan';
        
        elseif (objectData{k,3}.ZLimits(2) < 0) && (objectData{k,1} == "can" ) %%% needs to be tested Zlimits might not look right
            objectData{k,1} = 'hCan';
        
        else 
            continue;
        end
    end

end