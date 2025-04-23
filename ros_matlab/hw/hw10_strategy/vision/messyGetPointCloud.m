function [ptCloud_tform, ptCloud, R_T_C, C_T_R] = messyGetPointCloud(optns)
    % Access rosClass
    r = optns{'rHandle'};
    
    % Get point cloud    
    pc = receive(r.pt_cloud_sub);

    % Get transform from base_link to camera_depth_link from ROS TF tree at point cloud timestamp.
    % ROS buffers position data for a few seconds, allowing retrieval of previous positions.
    % This eliminates the need to remain stationary while waiting for transforms.
    % Without buffering, rapid movement would result in incorrect transforms and misaligned
    % point clouds unless artificial delays were added to the code.
    R_T_C_tfmsg = getTransform(r.tftree,'base_link', pc.Header.FrameId, pc.Header.Stamp, 'Timeout', r.tf_listening_time);
    R_T_C = rosReadTransform(R_T_C_tfmsg, OutputOption="single"); % Converts rosmsg to MATLAB transform
    C_T_R = inv(R_T_C); % No need to ask ros for the inverse transform, just invert the matrix.

    % Extract xyz points
    xyz = rosReadXYZ(pc,"PreserveStructureOnRead",true);
    rgb = rosReadRGB(pc,"PreserveStructureOnRead",true);
    % Points -> MATLAB pointCloud object
    ptCloud = pointCloud(xyz, "Color", rgb); 
    
    % Transform point cloud to base_link
    tform = rigidtform3d(R_T_C);

    ptCloud_tform = pctransform(ptCloud,tform);
end