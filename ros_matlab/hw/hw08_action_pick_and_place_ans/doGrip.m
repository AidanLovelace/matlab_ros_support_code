function [res,state] = doGrip(type,optns,doGripValue )
%--------------------------------------------------------------------------
% Tell gripper to either pick or place via the ros gripper action client
%
% Input: type (string) - 'pick' or 'place'
% Output: actoin result and state
%--------------------------------------------------------------------------

    %% Init
    r = optns{'rHandle'};    
    

    % Create a gripper goal action message
    grip_msg = rosmessage(r.grip_action_client);

    %% Testing if setting FeedbackFcn to 0 minimizes the loss connection
    r.grip_action_client.FeedbackFcn = [];
    r.grip_action_client.ResultFcn = [];

    %% Set Grip Pos by default to pick / close gripper
    if nargin==3
        type = 'pick';
        gripPos = doGripValue;
    end

    % Modify it if place (i.e. open)
    if strcmp(type,'place')
        gripPos = 0;           
    end

    %% Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg,optns);

    %% Pending: Check if fingers already at goal
    % Get current finger position
    % Compare with with goal
    % If so, do not call action
    
    %% Send action goal
    disp('Sending grip goal...');

    try waitForServer(r.grip_action_client,2)
        disp('Connected to Grip server. Moving fingers...')
        [res,state,status] = sendGoalAndWait(r.grip_action_client,grip_goal);
    catch
        % Re-attempt
        disp('No server..? Trying again...');
        [res,state,status] = sendGoalAndWait(r.grip_action_client,grip_goal);
    end    
end