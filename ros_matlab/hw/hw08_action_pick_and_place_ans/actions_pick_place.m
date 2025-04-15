% Pick and place

%% 00 Connect to ROS (use your own masterhost IP address)
clc
clear
rosshutdown;
pause(2);       

%% Set IP address for master and node:
masterHostIP = "192.168.56.101";
nodeHostIP = "192.168.56.1";
rosinit(masterHostIP, 11311, "NodeHost",nodeHostIP);

%% ROS Class handle

% r will contains all publisher/subscriber/service/action/kinematic/tf info
disp("Creating Robot Handle...");
r = rosClassHandle_UR5e;

%% Options 

% Create global dictionary. Passed to all functions. Includes robot handle
keys   = ["debug", "toolFlag", "traj_steps", "z_offset", "traj_duration", "frameAdjustmentFlag", "toolAdjustmentFlag", "toolAdjustment", "rHandle"];
values = {      0,          0,            1,       0.09,               2,                     1,                    1,            0.165,         r};

% Instantiate the dictionary: values can be access via {}, i.e. optns{'key'}
disp("Creating dictionary...");
optns = dictionary(keys,values);

%% 02 Go Home 

% Always move arm to start position and open fingers
disp('Going home...');  % Always use display commands to inform reader of what is happening

% Open fingers and move arm to qr or qz start configuration 
goHome('qr', optns);    

%% 03 Reset the simulation

disp('Resetting the world...');
resetWorld(optns);      % reset models through a gazebo service

%% 04 Get Model Poses

type = 'gazebo'; % gazebo, ptcloud, cam, manual
disp('Getting object goal pose(s)...')

% Get models from Gazebo
models = getModels(optns);

% Number of models to pick (you can hard code or randomize)
n = 1; % n = randi([3 25]);

% Manual Specification of fixed objects (may change from year-to-year)
rCan1 = [0.4, -0.5, 0.14, -pi/2, -pi 0];
rCan2 = [0.018, 0.66, 0.25, -pi/2, -pi 0];
rCan3 = [0.8, -0.03, 0.15, -pi/2, -pi, 0];
model_pos = [rCan1;rCan2;rCan3];

% Bin locations
greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];

%% 05 Pick up Objects
% Create a loop to do:
%   1. Get model pose in homogenous transformation format,
%   2. Execute a top-down pick
%   3. Place in correct bin
%   5. Go home

% Set the rate at which this loop should run
rate = rosrate(10);

% For Gazebo retrieved poses
if strcmp(type,'gazebo')

    % For n models to pick up
    for i=1:n

        %% 05.1 Get Model Pose
        
        % 05.1.1 Get Model Name
        model_name = models.ModelNames{23+randi([7,8])};

        % 05.1.2 Get Model pose
        fprintf('Picking up model: %s \n',model_name);
        get_robot_gripper_pose_flag = 0; % 0 - no model of fingers available
        [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name,get_robot_gripper_pose_flag,optns);
       
        % 05.2 Pick Model
        strategy = 'topdown';               % Assign strategy: topdown, direct
        ret = pick(strategy, mat_R_T_M,optns); % Can have optional starting opse for ctraj like: ret = pick(strategy, mat_R_T_M,mat_R_T_G);
        
        % 05.3 Place
        if ~ret   % If no errors, continue to place in bin       

            % 05.3.1 Set pose of green bin
            place_pose = set_manual_goal(greenBin);
            
            % Move to Bin 
            disp('Attempting to place in bin...')
            strategy = 'topdown';            
            ret = moveToBin(strategy,mat_R_T_M,place_pose,optns);
        end

        % 05.4 Return to home
        if ~ret     % If no errors

            % Move to ready position
            ret = moveToQ('qr',optns);
        end

        % Control loop
        waitfor(rate);
    end

% Manual pose
elseif strcmp(type,'manual')
    for i=1:n

        %% 05.1 Get Model Pose
        
        % 05.1.1 Get Model Name
        model_name = models.ModelNames{23+i};

        % 05.1.2 Get Model pose
        fprintf('Picking up model: %s \n',model_name);
        [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name,get_robot_gripper_pose_flag,optns);
       
        % 05.2 Pick Model
        strategy = 'topdown';            % Assign strategy: topdown, direct
        ret = pick(strategy, mat_R_T_M); % Can have optional starting opse for ctraj like: ret = pick(strategy, mat_R_T_M,mat_R_T_G);
        
        % 05.3 Place
        if ~ret   % If no errors, continue to place in bin       

            % 05.3.1 Set pose of green bin
            place_pose = set_manual_goal(greenBin);
            
            % Move to Bin 
            disp('Attempting to place in bin...')
            strategy = 'topdown';            
            ret = moveToBin(strategy,mat_R_T_M,place_pose);
        end

        % 05.4 Return to home
        if ~ret     % If no errors

            % Move to ready position
            ret = moveToQ('qr');
        end

        % Control loop
        waitfor(rate);
    end
else
    disp("Error: unknown input (type)");
end

%% Finalize everything
goHome('qr',optns)
rosshutdown;
clear