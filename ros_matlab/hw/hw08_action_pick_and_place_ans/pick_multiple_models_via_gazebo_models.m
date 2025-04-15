% Pick and place

%% 00 Connect to ROS (use your own masterhost IP address)
clc
clear
rosshutdown;

pause(2);       % Check if more down time helps diminish connection errors
masterhostIP = "192.168.64.129";
rosinit(masterhostIP)

% if exist('r','var') && isvalid(r)
%     disp('Robot handle exists and is valid.')
% else
%     r = rosClassHandle;
% end
r = rosClassHandle;
%y = yoloClassHandle;
keys   = ["debug", "toolFlag", "traj_steps", "x_offset", "y_offset", "z_offset", "traj_duration", "frameAdjustmentFlag", "toolAdjustmentFlag", "toolAdjustment", "rHandle"];
values = {      0,          0,            1,          0,          0,        0.2,               2,                     1,                    1,            0.165,         r};
optns = dictionary(keys,values);

%% 02 Go Home
disp('Going home...');
goHome('qtest',optns);    % moves robot arm to a qr or qz start config

resetWorld;      % reset models through a gazebo service
playingAround(optns);

%% 03 Get Pose from Gazebo Models
disp('Getting goal...') 
model_names = {'rCan3','yCan1','yCan3','gCan4','rCan2','gCan3','gCan1','rCan1','gCan2'};
mod_sz = length(model_names);

% Loop through them.
rate = rosrate(1);
for i=1:mod_sz
    nm = model_names{i};
    fprintf('Picking up model: %s \n',nm);
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(nm,0,optns);

    %% 04 Pick Model
    % Assign strategy: topdown, direct
    strategy = 'topdown';
    ret = pick(strategy, mat_R_T_M, optns); % Can have optional starting opse for ctraj like: ret = pick(strategy, mat_R_T_M,mat_R_T_G);
    
    %% 05 Place
    if ~ret
        disp('Attempting place...')
        
        % Set bin goals
        greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0]; % left
        blueBin  = [-0.4,  0.45, 0.25, -pi/2, -pi 0]; % right
        
        % Set place pose to blue bin only if it is a bottle
        if contains(nm,'Bottle')
            goal_pose = blueBin; 
        else
            goal_pose = greenBin;
        end

        place_pose = set_manual_goal(goal_pose);

        strategy = 'topdown';
        fprintf('Moving to bin...');
        ret = moveToBin(strategy,mat_R_T_M,place_pose,optns);
    end
    
    %% Return to home
    if ~ret
        ret = moveToQ('qr',optns);
    end

    % Control loop
    waitfor(rate);
end

    