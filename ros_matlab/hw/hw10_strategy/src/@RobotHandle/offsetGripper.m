function gripper_pose = offsetGripper(this, gripper_pose, direction, offset, ang, ang_offset)
%--------------------------------------------------------------------------
% gripper_pose = displace_gripper(gripper_pose,direction,offset)
% Displaces the gripper. Assumes matlab coordinate frame for UR5e.
%
% Inputs:
% gripper_pose: [4x4]       - current gripper pose
% optns:        (dict)      - options
% direction:    string      - 'f', 'b', 'r', 'l', 'c' for forward, backwards, right, left, center looking towards the back of the robot.
% offset:       double'     - how much do you want to move the gripper by in linear motion. default 0.10
% ang:          double      - flag to turn ON or OFF, rotational adjustments
% ang_offset    double      - by how many radians to rotate the camera
%
% Output:
% gripper_pose [4x4] - homogeneous transformation.
%--------------------------------------------------------------------------

    vertical_offset = 0.2; % move down to get more detail

    % Default options
    if nargin == 2
        direction = 'f';
        offset = 0.10;
        ang = 0; % no angle offset
        ang_offset = 0;

    elseif nargin == 3
        offset = 0.10;
        ang = 0; % no angle offset
        ang_offset = 0;

    elseif nargin == 4
        ang = 0; % no angle offset
        ang_offset = 0;
    else %nargin == 5
           ang_offset = pi/6;
    end

    % Displace according to direction

    % Move forward 10cm from central position
    if strcmpi(direction, 'f')
        gripper_pose(2,4) = gripper_pose(2,4) + offset;
        gripper_pose(3,4) = gripper_pose(3,4) - vertical_offset; % move down to get more detail
        this.moveToPose(gripper_pose, 1);

        % Adjust angles once and keep fixed
        if ang
            ori = trotx(-ang_offset);
            gripper_pose = gripper_pose*ori;
            this.moveToPose(gripper_pose, 1);
        end

    % Move back 10cm from central position
    elseif strcmpi(direction, 'b')
        gripper_pose(2,4) = gripper_pose(2,4) - offset;
        gripper_pose(3,4) = gripper_pose(3,4) - vertical_offset; % move down to get more detail
        this.moveToPose(gripper_pose);
        % Adjust angles once and keep fixed
        if ang
            ori = trotx(ang_offset);
            gripper_pose = gripper_pose*ori;
            this.moveToPose(gripper_pose, 1);
        end

    % Move left (facing towards the back of robot, i.e. matlab -x direction)
    elseif strcmpi(direction, 'l')
        gripper_pose(1,4) = gripper_pose(1,4) - offset;
        gripper_pose(3,4) = gripper_pose(3,4) - vertical_offset; % move down to get more detail
        this.moveToPose(gripper_pose, 1);

        % Adjust angles once and keep fixed
        if ang
            ori = troty(+ang_offset);
            gripper_pose = gripper_pose*ori;
            this.moveToPose(gripper_pose, 1);
        end

    % Move right
    elseif strcmpi(direction, 'r')
        gripper_pose(1,4) = gripper_pose(1,4) + offset;
        gripper_pose(3,4) = gripper_pose(3,4) - vertical_offset; % move down to get more detail
        this.moveToPose(gripper_pose, 1);

        % Adjust angles once and keep fixed
        if ang
            ori = troty(-ang_offset);
            gripper_pose = gripper_pose*ori;
            this.moveToPose(gripper_pose, 1);
        end
    elseif strcmpi(direction, 'u')
        gripper_pose(3,4) = gripper_pose(3,4) + offset;
        gripper_pose(3,4) = gripper_pose(3,4) - vertical_offset; % move down to get more detail
        this.moveToPose(gripper_pose, 1);

        % Adjust angles once and keep fixed
        if ang
            ori = troty(-ang_offset);
            gripper_pose = gripper_pose*ori;
            this.moveToPose(gripper_pose, 1);
        end

    else % 'c'
        this.moveToPose(gripper_pose, 1);
    end
end