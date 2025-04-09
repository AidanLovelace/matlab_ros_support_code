
    %% 00 Connect to ROS (use your own masterhost IP address)
    clc
    clear
    rosshutdown;
    pause(2);       
    
    %% Set IP address for master and node:
    masterHostIP = "192.168.56.101";
    nodeHostIP = "192.168.56.1";
    rosinit(masterHostIP, 11311, "NodeHost",nodeHostIP);
    
    %% 01 ROS Class handle
    
    % r will contains all publisher/subscriber/service/action/kinematic/tf info
    disp("Creating Robot Handle...");
    r = rosClassHandle_UR5e;

    %% 02 Create Options 
    
    % Create global dictionary. Passed to all functions. Includes robot handle
    keys   = ["debug", "toolFlag", "traj_steps", "z_offset", "traj_duration", "frameAdjustmentFlag", "toolAdjustmentFlag", "toolAdjustment", "rHandle"];
    values = {      1,          0,            1,       0.09,               2,                     1,                    1,            0.165,         r};
    
    % Instantiate the dictionary: values can be access via {}, i.e. optns{'key'}
    disp("Creating dictionary...");
    optns = dictionary(keys,values);
    
    %% 03 Go Home 
    
    % Always move arm to start position and open fingers
    disp('Going home...');  % Always use display commands to inform reader of what is happening
    
    % Open fingers and move arm to qr or qz start configuration 
    goHome('qr', optns);    
    
    %% 04 Reset the simulation
    
    disp('Resetting the world...');
    resetWorld(optns);      % reset models through a gazebo service       

    %% 05 Compute Pose of Visible Objects

    % In more advanced code later we will create zones. For now leave empty
    zoneInspect = ''; 

    % Get labeled images and metadata of the zone
    [bboxes, ~, labeled, numOfObjects, myImg, annotatedImage] = getLabeledImg(zoneInspect,optns);
    
    % Capture a point cloud of the zone
    [ptCloud_pic, nonPlane_pic, ptCloud_table, base_to_cam_pose, cam_to_base_pose] = getMergedPTC(zoneInspect,optns); 
    
    % Get the pose of each detected object
    objectData = getObjectData(ptCloud_pic, nonPlane_pic, myImg, bboxes, numOfObjects, base_to_cam_pose, cam_to_base_pose, labeled);