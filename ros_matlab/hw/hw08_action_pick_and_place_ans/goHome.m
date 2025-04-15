function [ret,q,state,status] = goHome(config,optns)
    %----------------------------------------------------------------------
    % This functions uses the FollowJointTrajectory service to send desired
    % joint angles that move the robot arm to a home or ready
    % configuration. 
    % 
    % The angles are hard-coded. 
    % 
    % Expansion:
    % - Possible to expand to different desirable configurations and have an
    % input string make the selection. 
    % - Check if robot already at desired position. Then skip action calls.
    %
    % Inputs:
    % config (string): 'qr', 'qz' or other. 
    %                   qz = zeros(1,6)
    %                   qr = [0 0 pi/2 -pi/2 0 0]
    % optns (dict) : arguments for class
    %
    % Outputs
    % resultMsg (struc) - Result message
    % state [char] - Final goal state
    % status [char]- Status text
    %----------------------------------------------------------------------
    % Open Fingers
    disp('Opening fingers...')
    doGrip('place', optns);

    % Move arm according to config
    if nargin == 0
        [ret,q] = moveToQ('qr', optns);
    else
        [ret,q] = moveToQ(config, optns);
    end
end