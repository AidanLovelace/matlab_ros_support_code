function jointAngles = getRobotJointLimits(r,optns)
%--------------------------------------------------------------------------
%jointAngles = getRobotJointLimits(r,optns)
% Returns jointAngles structure that is indexed by the name of the joint as a mx2 structure, where m is the number of
% joints and there is a lower limit and an upper limit.
%
% When the name of the joint is provided as a field to the structure:
%   jointAngles.(jointName) the structure returns the lower and upper
%   limits for that joint.
% Input:    r - robot model for matlab
%           optns: (dict) - in case debug flag wants to be activated
% Output:   
%           jointAngles
%--------------------------------------------------------------------------

    % Create a structure to hold the joint angles and limits
    jointAngles = struct();

    % Access joint information from the robot model (excluding the base link)
    for i = 2:numel(r.Bodies)
        joint = r.Bodies{i}.Joint;
    
        % Check if the joint is revolute (has joint limits)
        if strcmpi(joint.Type, 'revolute')
            jointName = joint.Name;
            jointLimits = joint.PositionLimits; 
            jointAngles.(jointName) = jointLimits; % Store limits as 1x2 vector
    
            if optns("debug")
                fprintf('%s limits: %.2f to %.2f radians\n', jointName, jointLimits(1), jointLimits(2));
            end
        end

    end

end