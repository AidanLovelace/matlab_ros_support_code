function [ptCloud_tform, ptCloud, base_to_cam_pose, cam_to_base_pose] = messyGetPointCloud(optns)

    % Access rosClass
    r = optns{'rHandle'};
    
    %% Get point cloud
    disp("Creating point cloud...")
    ds = r.pt_cloud_sub;
    pc = receive(ds,2);

    % Extract xyz points
    xyz = rosReadXYZ(pc,"PreserveStructureOnRead",true);
    
    % Extract point cloude
    ptCloud = pointCloud(xyz); 



    %% Get base_link to camera_depth_link transform
    tftree       = rostf('DataFormat','struct');     
    base         = 'base_link';
    end_effector = pc.Header.FrameId; %'camera_depth_link';
    
    % Compute the ROS/Gazebo transform from base_link to camera_depth_link
    waitForTransform(tftree, base, end_effector);
    base_to_camera = getTransform(tftree, base, end_effector, rostime('now'),'Timeout', r.tf_listening_time);

    %% base_to_camera 4x4 transform matrix
    % Using base to camera transformation gotten from gazebo when the point
    % cloud was extracted, making a pos variable from its translation.
    % FIXME - can use existing function to do this. 
    pos = [ base_to_camera.Transform.Translation.X, ... 
            base_to_camera.Transform.Translation.Y, ...
            base_to_camera.Transform.Translation.Z]; 
    
    % Using base_to_camera rotation and formatting into q    
    q = UnitQuaternion(base_to_camera.Transform.Rotation.W, ...
                       [base_to_camera.Transform.Rotation.X, ...
                        base_to_camera.Transform.Rotation.Y, ...
                        base_to_camera.Transform.Rotation.Z]);
    
    % Compute the 4x4
    q.T;
    base_to_cam_pose = transl(pos) * q.T;   % now a 4x4

    %% Get camera_depth_link to base_link
    waitForTransform(tftree, end_effector, base);
    camera_to_base = getTransform(tftree, end_effector, base, rostime('now'),'Timeout', r.tf_listening_time);

    %% camera_to_base 4x4 transform matrix
    % Using camera to base transformation gotten from gazebo when the point
    % cloud was extracted, making a pos variable from its translation.
    pos = [ camera_to_base.Transform.Translation.X, ... 
            camera_to_base.Transform.Translation.Y, ...
            camera_to_base.Transform.Translation.Z]; 
    
    % Using base_to_camera rotation and formatting into q    
    q = UnitQuaternion(camera_to_base.Transform.Rotation.W, ...
                       [camera_to_base.Transform.Rotation.X, ...
                        camera_to_base.Transform.Rotation.Y, ...
                        camera_to_base.Transform.Rotation.Z]);
    
    % Compute the 4x4
    q.T;
    cam_to_base_pose = transl(pos) * q.T;   % now a 4x4    
    
    %% Transform point cloud to base_link
    tform = rigidtform3d(base_to_cam_pose);

    ptCloud_tform = pctransform(ptCloud,tform);
    
    %% Create point cloud visually **TransformedPtCloud (tpc)

    disp("Plotting point cloud with respect to base_link...")
    
    planeThickness = .001;
    normalVector = [0,0,1];
    maxPlaneTilt = 5;
    [param_tpc, planeIdx_tpc, nonPlaneIdx_tpc] = pcfitplane(ptCloud_tform, planeThickness, normalVector, maxPlaneTilt);
    plane_tpc = select(ptCloud_tform, planeIdx_tpc);
    nonPlane_tpc = select(ptCloud_tform, nonPlaneIdx_tpc);

    % show plane
    %figure,pcshow(plane_tpc,'ViewPlane','XY');axis on;
    % show items
    %figure,pcshow(nonPlane_tpc,'ViewPlane','XY');axis on;
    %xlabel("X"); ylabel("Y"); zlabel("Z"); title("nonPlane of base link recent point cloud");
    %% Create point cloud of camera_depth_link reference visually
    % disp("Plotting point cloud with respect to camera_depth_link...")
    % planeThickness = .001;
    % normalVector = [0,0,1];
    % maxPlaneTilt = 5;
    % [param, planeIdx, nonPlaneIdx] = pcfitplane(ptCloud, planeThickness, normalVector, maxPlaneTilt);
    % plane = select(ptCloud, planeIdx);
    % nonPlane = select(ptCloud, nonPlaneIdx);
    % 
    % % show plane
    % %figure,pcshow(plane,'ViewPlane','XY');axis on;
    % % show items
    % figure,pcshow(nonPlane,'ViewPlane','XY');axis on;
    % xlabel("X"); ylabel("Y"); zlabel("Z"); title("nonPlane of camera_depth_link recent point cloud");
    %% Create non-plane mask
    % get size of image
    %[m,n,~] = size(myImg);
    % m = 480;
    % n = 640;
    % % create variable with dimensions of image
    % nonPlaneMask_tpc = zeros(m,n);
    % % make variable only a column
    % nonPlaneMask_tpc = nonPlaneMask_tpc(:);
    % % use nonPlaneIdx to fill nonPlaneMask with useful data
    % nonPlaneMask_tpc(nonPlaneIdx_tpc) = 1;
end