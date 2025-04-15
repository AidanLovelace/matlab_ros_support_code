function [res,q] = moveToQ(config,optns,custom)
% moveToQ 
% Move to joint configuration prescribed by config
% 
% Expansion/TODO: 
% - Check if robot already at desired position. Then skip action calls.
%
% Inputs
% config (string):    qr, qz... can expand in future
%
% Outputs:
% res (bool): 0 indicates success, other failure.
%----------------------------------------------------------------------

    r = optns{'rHandle'};  

    % Get latest joint_states
    ros_cur_jnt_state_msg = receive(r.joint_state_sub,1);
    
    % Get steps/duration
    traj_steps = optns{'traj_steps'};
    traj_duration = optns{'traj_duration'};
    
    % Create action goal message from client
    traj_goal = rosmessage(r.pick_traj_act_client); 

    %% Cancelling feedback/result fncn due to 2025 bug in matlab-ros
    r.pick_traj_act_client.FeedbackFcn = [];      
    r.pick_traj_act_client.ResultFcn = [];
    % Set to qr
    if nargin == 0 || strcmp(config,'qr')
        % Ready config
        q = [0 0 pi/2 -pi/2 0 0];
    
    % Set to qz
    elseif(strcmp(config,'qz'))
        q = zeros(1,6);
    
    % Set to qtest
    elseif(strcmp(config,'qtest'))
        q = [0 pi/4 pi/4 -pi/2 0 0];
    
    % Custom
    elseif strcmp(config, "custom") ||  nargin == 3
        q = custom;
    
    % Default to qr ready config
    else
        q = [0 0 pi/2 -pi/2 0 0];
    
    end

    %% Convert to ROS waypoint
    traj_goal = convert2ROSPointVec(q,...
                                    ros_cur_jnt_state_msg.Name,...
                                    traj_steps,...
                                    traj_duration,...
                                    traj_goal,...
                                    optns);
    
    %% Send ros trajectory with traj_steps
    if waitForServer(r.pick_traj_act_client)
        disp('Connected to Arm server. Moving arm...')
        [res,state,status] = sendGoalAndWait(r.pick_traj_act_client,traj_goal);
    else
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(r.pick_traj_act_client,traj_goal);
    end
    
    % Extract result
    res = res.ErrorCode;
    
end