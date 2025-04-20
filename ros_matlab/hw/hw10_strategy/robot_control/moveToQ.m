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
    logPrint(8, 'moveToQ', 1, '', "Received current joint state");
    
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
        logPrint(8, 'moveToQ', 1, '', "Moving to right angle position 'qr'");
        % Ready config
        q = [0 0 pi/2 -pi/2 0 0];
        
        % Set to qz
    elseif(strcmp(config,'qz'))
        logPrint(8, 'moveToQ', 1, '', "Moving to zero joint angle position 'qz'");
        q = zeros(1,6);
        
        % Set to qtest
    elseif(strcmp(config,'qtest'))
        logPrint(8, 'moveToQ', 1, '', "Moving to test position 'qtest'");
        q = [0 pi/4 pi/4 -pi/2 0 0];
        
        % Custom
    elseif strcmp(config, "custom") ||  nargin == 3
        logPrint(8, 'moveToQ', 1, '', "Moving to custom position");
        q = custom;
        
        % Default to qr ready config
    else
        logPrint(8, 'moveToQ', 1, '', "Config not recognized. Moving to default 'qr' position");
        q = [0 0 pi/2 -pi/2 0 0];
    end
    logPrint(8, 'moveToQ', 2, '', "q = [%.2f %.2f %.2f %.2f %.2f %.2f]", q(1),q(2),q(3),q(4),q(5),q(6));
    

    %% Convert to ROS waypoint
    traj_goal = convert2ROSPointVec(q,...
                                    ros_cur_jnt_state_msg.Name,...
                                    traj_steps,...
                                    traj_duration,...
                                    traj_goal,...
                                    optns);
    
    %% Send ros trajectory with traj_steps
    if waitForServer(r.pick_traj_act_client)
        logPrint(8, 'moveToQ', 1, '', "Server Responded. Sending trajectory to action server.");
        [res,state,status] = sendGoalAndWait(r.pick_traj_act_client,traj_goal);
    else
        logPrint(8, 'moveToQ', 1, '', "Server Did Not Respond. Sending trajectory to action server anyway.");
        [res,state,status] = sendGoalAndWait(r.pick_traj_act_client,traj_goal);
    end
    
    logPrint(8, 'moveToQ', 2, '', "Result State: %s", state);
    logPrint(8, 'moveToQ', 2, '', "Result Status: %s", status);
    logPrint(8, 'moveToQ', 2, '', "Result Message: %s", res.MessageType);
    logPrint(8, 'moveToQ', 3, '', "SUCCESSFUL: %s", string(res.SUCCESSFUL));
    logPrint(8, 'moveToQ', 3, '', "Error Code: %s", string(res.ErrorCode));
    logPrint(8, 'moveToQ', 3, '', "ErrorString: %s", string(res.ErrorString));
    
    % Extract result
    res = res.ErrorCode;
end