function [xyz,theta,ptCloud_vec,scene_pca_vec] = betterObjectPoses(ptCloud_tform_cam, rbgImage, bboxes, gridDownsample, base_to_cam_pose)
% the base is findObjectPoses but it is now tweaked by Gracelyn to better
% suit to use of merged point clouds

% declaring and allocating variables
numObjects = size(bboxes,1);
ptCloud_vec = cell(numObjects, 1);
scene_pca_vec = cell(numObjects, 1);
theta = [];
xyz =[];

% finding the xyz limits of the point cloud
pc_xmin = ptCloud_tform_cam.XLimits(1);
pc_xmax = ptCloud_tform_cam.XLimits(2);
pc_ymin = ptCloud_tform_cam.YLimits(1);
pc_ymax = ptCloud_tform_cam.YLimits(2);
pc_zmin = ptCloud_tform_cam.ZLimits(1);
pc_zmax = ptCloud_tform_cam.ZLimits(2);

[m,n,~] = size(rbgImage); % get the size of image

for idx= 1:numObjects

    % calculate the pixel scale for width and height
    xpixel_base_scale = (pc_xmax - pc_xmin)/n;
    ypixel_base_scale = (pc_ymax - pc_ymin)/m;
    
    % Get the bouding box edges with respect to the image size/pixels
    xincrease = bboxes(idx,3)/6;
    yincrease = bboxes(idx,4)/6;

    left_column = bboxes(idx,1) - xincrease;
    right_column = bboxes(idx,1) + bboxes(idx,3) + xincrease;

    top_row = bboxes(idx,2) - yincrease;
    bottom_row = bboxes(idx,2) + bboxes(idx,4) + yincrease;

    % Clip
    if left_column < 1
        left_column = 1;
    end
    if right_column > n
        right_column = n;
    end
    if top_row < 1
        top_row = 1;
    end
    if bottom_row > m
        bottom_row = m;
    end

    % Use bounding box locations on the image and translate them to the

    % base link reference
    bbox_roi_xmin = pc_xmin + (left_column*xpixel_base_scale);
    bbox_roi_xmax = pc_xmin + (right_column*xpixel_base_scale);
    bbox_roi_ymin = pc_ymin + (top_row*ypixel_base_scale);
    bbox_roi_ymax = pc_ymin + (bottom_row*ypixel_base_scale);

    % the roi created by the bounding box with respect to the point cloud
    bbox_roi = [bbox_roi_xmin bbox_roi_xmax bbox_roi_ymin bbox_roi_ymax pc_zmin pc_zmax];
    % find the point cloud points within the bounding box
    instanceMask = findPointsInROI(ptCloud_tform_cam,bbox_roi);
    pt_instance = select(ptCloud_tform_cam,instanceMask);

    % transform the point cloud back to base_link reference frame
    tform_to_base = rigidtform3d(base_to_cam_pose);
    pt_instance_base = pctransform(pt_instance,tform_to_base);
    pt_instance_base = pcdownsample(pt_instance_base, 'gridAverage', gridDownsample); % makes the voxel size bigger
    % remove partial point cloud of other object
    [lab_instance, num_instance] = pcsegdist(pt_instance_base, gridDownsample * sqrt(3));

    % finding which point cloud segment has the most points in it
    c = [];
    for j = 1:num_instance
        c(j) = nnz(lab_instance == j);
    end
    [~, max_labelJ] = max(c);

    % keep the point cloud segment with the most points in it
    ptCloud_vec{idx} = select(pt_instance_base, find(lab_instance==max_labelJ));
    ptScene = ptCloud_vec{idx};
    % PCA
    [coeff, score, latent] = pca(ptScene.Location);

    if (det(coeff) < 0)  % det = -1 means rigyht-handed system, so transforms into left-handed system
        coeff(:, 2) = -coeff(:, 2);
    end

    %compute scene each object PCA
    centroid= mean(ptScene.Location);
    coeff = align2ndAxis(ptScene, coeff, centroid);%align axis by using point cloud density
    coeff = align3rdAxis(ptScene, coeff, centroid);
    [U,V,W] = makeUVWfromCoeff(coeff);

    % make pca struct
    scene_pca.coeff = coeff;
    scene_pca.score = score;
    scene_pca.latent = latent;
    scene_pca.centroid = centroid;
    scene_pca.eulZYX = rotm2eul(coeff);
    scene_pca.UVW = [U,V,W];
    scene_pca_vec{idx} = scene_pca;
    majorAxis = [U(1), V(1), W(1)];

    %This calculates the angle between the positive x-axis ([ 1 0 0]) and the major axis of the object in an anti-clockwise direction
    theta(idx) = atan2d(dot([0 0 1],cross([ 1 0 0],majorAxis)),dot([ 1 0 0],majorAxis));
    if (theta(idx)<0)
        theta(idx) = 180 + theta(idx);
    end
    
    roi = [centroid(1)-0.01 centroid(1)+0.01 centroid(2)-0.01 centroid(2)+0.01 -inf inf];
    indices = findPointsInROI(ptScene,roi);
    pt_surface_patch = select(ptScene,indices);
    z = mean(pt_surface_patch.Location(:,3));
    xyz(idx,:) = [centroid(1) centroid(2) z];
end
end

%--------------------------------------------------------------------------
% Helper functions
%--------------------------------------------------------------------------
function coeff_aligned = align2ndAxis(ptCloud, coeff, center)
%--------------------------------------------------------------------------
% Function: align2ndAxis
%
% Aligns the second PCA axis based on point density distribution along the
% Y-axis. Ensures the second axis points toward the direction with higher
% point density.
%
% Inputs:
% - ptCloud : Input point cloud of the object.
% - coeff   : Initial PCA coefficient matrix.
% - center  : Centroid of the point cloud.
%
% Output:
% - coeff_aligned : Aligned PCA coefficient matrix ensuring correct Y-axis
%                   orientation.
%--------------------------------------------------------------------------

    % Shift point cloud so centroid is at the origin for analysis
    ptCloud_aligned = pctransform(ptCloud, rigid3d(eye(3), -center));

    % Rotate point cloud based on initial PCA coefficients for alignment
    ptCloud_aligned = pctransform(ptCloud_aligned, rigid3d(coeff, [0 0 0]));

    % Extract Y-coordinates of transformed points for horizontal analysis
    points_y = ptCloud_aligned.Location(:, 2);

    % Separate points into those on positive and negative sides of Y-axis
    posy = points_y > 0;

    % Compute squared distances of points on positive and negative sides
    moments_pos = sum(points_y(posy) .^ 2);    % Positive side
    moments_neg = sum(points_y(~posy) .^ 2);   % Negative side

    % Prepare an alternate PCA coefficient matrix with inverted axes
    coeff_rot = coeff;
    coeff_rot(:, 2) = -coeff(:, 2);    % Flip second principal axis
    coeff_rot(:, 3) = -coeff(:, 3);    % Flip third principal axis

    % Determine alignment based on which side has greater point density
    if (moments_pos > moments_neg)
        % Keep original orientation if positive side dominates
        coeff_aligned = coeff;
    else
        % Invert orientation if negative side dominates
        coeff_aligned = coeff_rot;
    end
end


%--------------------------------------------------------------------------
function coeff_aligned = align3rdAxis(ptCloud, coeff, center)
%--------------------------------------------------------------------------
% Function: align3rdAxis
%
% Aligns the third PCA axis (typically the vertical Z-axis) based on point
% density along the vertical direction. Ensures consistent vertical
% orientation for the object's pose.
%
% Inputs:
% - ptCloud : Input point cloud of the object.
% - coeff   : Initial PCA coefficient matrix.
% - center  : Centroid of the point cloud.
%
% Output:
% - coeff_aligned : Aligned PCA coefficient matrix ensuring correct vertical
%                   orientation.
%--------------------------------------------------------------------------

    % Shift point cloud so centroid is at the origin for analysis
    ptCloud_aligned = pctransform(ptCloud, rigid3d(eye(3), -center));

    % Rotate point cloud based on initial PCA coefficients for alignment
    ptCloud_aligned = pctransform(ptCloud_aligned, rigid3d(coeff, [0 0 0]));

    % Extract Z-coordinates of transformed points for vertical analysis
    points_z = ptCloud_aligned.Location(:, 3);

    % Separate points into those above and below the horizontal plane (Z = 0)
    posy = points_z > 0;

    % Compute squared distances of points above and below the horizontal plane
    moments_pos = sum(points_z(posy) .^ 2);    % Above plane
    moments_neg = sum(points_z(~posy) .^ 2);   % Below plane

    % Prepare an alternate PCA coefficient matrix with inverted axes
    coeff_rot = coeff;
    coeff_rot(:, 1) = -coeff(:, 1);    % Flip first principal axis
    coeff_rot(:, 3) = -coeff(:, 3);    % Flip third principal axis (vertical axis)

    % Determine alignment based on which side has greater point density
    if (moments_pos > moments_neg)
        % Keep original orientation if points above plane dominate
        coeff_aligned = coeff;
    else
        % Invert orientation if points below plane dominate
        coeff_aligned = coeff_rot;
    end
end


%--------------------------------------------------------------------------
function [U,V,W] = makeUVWfromCoeff(coeff)
%--------------------------------------------------------------------------
% Converts PCA coefficients into scaled 3D vectors for visualizing PCA axes.
%
% In point cloud visualization (pcshow), the axes you're plotting (with
% quiver3) are overlayed onto points which are typically very close to each
% other, often within millimeter-level distances.
%
% Multiplying by a small number (like 0.05) "scales" the PCA axis to a size 
% that fits nicely within the local dimensions of your visualized object. 
% It "extends" clarity rather than actual length, effectively adjusting it 
% from "way too long" to "just right" for visualization purposes.
%--------------------------------------------------------------------------
    
    % Scale down to match point cloud density
    visual_fitting = 0.05;

    % 1st principal axis
    U1 = coeff(1, 1) * visual_fitting;
    V1 = coeff(2, 1) * visual_fitting;
    W1 = coeff(3, 1) * visual_fitting;
    
    % 2nd principal axis
    U2 = coeff(1, 2) * visual_fitting;
    V2 = coeff(2, 2) * visual_fitting;
    W2 = coeff(3, 2) * visual_fitting;
    
    % 3rd principal axis
    U3 = coeff(1, 3) * visual_fitting;
    V3 = coeff(2, 3) * visual_fitting;
    W3 = coeff(3, 3) * visual_fitting;
    U = [U1;U2;U3];
    V = [V1;V2;V3];
    W = [W1;W2;W3];

end