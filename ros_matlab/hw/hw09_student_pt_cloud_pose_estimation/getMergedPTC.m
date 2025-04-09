function [ptCloud_pic, nonPlane_pic, ptCloud_world, base_to_cam_pose, cam_to_base_pose] = getMergedPTC(zoneInspect, optns) 
%--------------------------------------------------------------------------    
% Merges multiple point clouds from different gripper positions.
%
% Acquires the initial point cloud in the world frame, crops it to a
% desired region, and then iteratively moves the gripper to different
% locations to collect additional point clouds. The additional point clouds
% are merged with the initial point cloud, and the merged cloud is finally
% cropped to produce a picture point cloud.
%
%   Inputs:
%       zoneInspect   - A string specifying the inspection zone (e.g., 'Zone3').
%       optns         - Options structure containing parameters for point cloud 
%                       acquisition and robot control.
%
%   Outputs:
%       ptCloud_pic       - The cropped point cloud corresponding to the area of 
%                           interest (picture area).
%       nonPlane_pic      - The subset of ptCloud_pic that does not belong to the 
%                           dominant plane.
%       ptCloud_world     - The merged point cloud in the world frame.
%       base_to_cam_pose  - Transformation matrix from the base frame to the camera frame.
%       cam_to_base_pose  - Transformation matrix from the camera frame to the base frame.
%
%   Process Overview:
%       1. Acquire initial point cloud from the robot's sensors.
%       2. Crop the point cloud to isolate the region of interest.
%       3. Iteratively move the gripper to different locations to obtain additional 
%          point clouds, and merge them.
%       4. Crop the merged point cloud to the area corresponding to the image.
%       5. Fit a plane in the cropped cloud and separate planar from non-planar points.
%--------------------------------------------------------------------------

    % Extract point cloud and transforms in both directions
    [ptCloud_world, ~, base_to_cam_pose, cam_to_base_pose] = messyGetPointCloud(optns);
    
    %% Gather the x and y limits of this very first point cloud and store them
    %% TO_ENHANCE: convert to a function
    xlim_min = ptCloud_world.XLimits(1,1);
    xlim_max = ptCloud_world.XLimits(1,2);
    
    ylim_min = ptCloud_world.YLimits(1,1);
    ylim_max = ptCloud_world.YLimits(1,2);
    
    % Z-limits are hard-coded and subject to modification
    zlim_min = -0.13; zlim_max = 0.4;
    
    % From these limits create a region of interest (ROI)
    roi = [xlim_min xlim_max ylim_min ylim_max zlim_min zlim_max];  
    
    %% Crop point cloud wrt to z values
    indices = findPointsInROI(ptCloud_world,roi);
    ptCloud_world = select(ptCloud_world,indices);
    
    %% Update xyz limits given the new point cloud
    xlim_min = ptCloud_world.XLimits(1,1);
    xlim_max = ptCloud_world.XLimits(1,2);
    
    ylim_min = ptCloud_world.YLimits(1,1);
    ylim_max = ptCloud_world.YLimits(1,2);
    
    % Hard-coded z-limits
    zlim_min = -0.13; zlim_max = 0.4;

    % New ROI for object
    pic_roi = [xlim_min xlim_max ylim_min ylim_max zlim_min zlim_max];

    % Hard-coded ROI for table
    table_roi = [-0.3 2 -1.5 1.5 zlim_min zlim_max];
    
    %% Gripper pose
    mat_R_T_G = get_gripper_pose(optns);
    cur_gripper_location = get_gripper_pose(optns);

    %% Compute locations for the arm to move to: (f-forward, l-left, r-right, b-back)
    if strcmpi(zoneInspect, "Zone3")
        locations = {'f', 'l','r'};
   
    else   
        % We repeat locations. First four have no angle offset, last four will 
        locations = {'f', 'b', 'l','r',...
                 'f', 'b', 'l','r',};
    end
    
    %% Move the arm to each of these locations and take pc pic
    for iter = 1:length(locations)

        % a) Move arm to ith location: 

        % These gripper motions are planar (no gripper rotation)
        if iter < 5
            displaceG = displace_gripper(mat_R_T_G,optns,locations{iter},0.07);
        
        % These motions rotate the gripper inwards at end of displacement
        % to center object.         elseif iter > 5
             displaceGA = displace_gripper(mat_R_T_G,optns,locations{iter},0.07,1,0.05);
        end

        % b) Get point cloud (above) at that location
        pause(5);
        [ptCloud_recent, ~, ~, ~] = messyGetPointCloud(optns);

        % c) TODO: Merge ptCloud_world and ptCloud_recent (world+above) with a grid step of 0.001 and output to ptCloud_world
        ptCloud_world = 

        % d) Find relevant points from merged pt cloud
        indices = findPointsInROI(ptCloud_world,table_roi);

        % TODO: select indecs for ptCloud_world and output ptCloud_world
        ptCloud_world = 
        pause(5);

        %% e) Visualize
        if optns{'debug'}
            figure(2); 
            pcshow(ptCloud_world,'ViewPlane','XY');
            axis on;
        end
    end

    %% To resent move arm back to original position (per zone)
    moveTo(cur_gripper_location,optns);  
    
    %% Cropping the merged point cloud to the area of our picture

    % Compute indeces of points inside new ROI
    indices = findPointsInROI(ptCloud_world,pic_roi);

    % Keep point clouds with those indeces (plane + objects)
    ptCloud_pic = select(ptCloud_world,indices);
      
    %% Create nonPlane and plane objects that index points that below to those entities
    
    % Horizontal Plane parameters
    planeThickness = .001;
    normalVector = [0,0,1];
    maxPlaneTilt = 5;

    % TODO: Fit the horizontal plane given ptCloud_pic and the three arguments above.
    [param, planeIdx, nonPlaneIdx] = 

    % Create indexed entities
    plane_pic = select(ptCloud_pic, planeIdx);
    nonPlane_pic = select(ptCloud_pic, nonPlaneIdx);
    
    %% Show Merged Point Clouds
    disp("Plotting final merged point cloud for the subzone...")
    
    if optns{'debug'}
        figure(2),pcshow(plane_pic,'ViewPlane','XY');axis on;
        
        % TODO: show nonPlane point cloude with an XY View of the plane and axis on
        figure(3),
        
        % Labels
        xlabel("X"); ylabel("Y"); zlabel("Z"); title("Cropped merged point cloud wrt base link");
    end    
end