function [res,state] = setGripper(this, gripPos, duration)
%--------------------------------------------------------------------------
% Tell gripper to either pick or place via the ros gripper action client
%
% Input: type (string) - 'pick' or 'place'
% Output: actoin result and state
%--------------------------------------------------------------------------
    if nargin < 3
        duration = 0.5;
    end

    % Create a gripper goal action message
    grip_msg = rosmessage(this.gripControllerClient);

    grip_msg.Trajectory.JointNames = {'robotiq_85_left_knuckle_joint'};

    % Set Goal Tolerance: set type, name, and pos/vel/acc tolerance
    % Note: tolerances are considered as: waypoint +/- tolerance
    grip_msg.GoalTolerance = rosmessage('control_msgs/JointTolerance','DataFormat', 'struct');

    grip_msg.GoalTolerance.Name  = 'control_msgs/FollowJointTrajectoryAction';
    grip_msg.GoalTolerance.Position     = 0;
    grip_msg.GoalTolerance.Velocity     = 0.1;
    grip_msg.GoalTolerance.Acceleration = 0.1;

    % Time Stamp
    this.trajPts.TimeFromStart   = rosduration(duration, 'DataFormat','struct');

    % Position
    this.trajPts.Positions       = gripPos;
    this.trajPts.Velocities      = zeros(size(gripPos));
    this.trajPts.Accelerations   = zeros(size(gripPos));
    this.trajPts.Effort          = 0.5;

    % Copy trajPts --> gripGoal.Trajectory.Points
    grip_msg.Trajectory.Points = this.trajPts;

    [res, state, ~] = sendGoalAndWait(this.gripControllerClient, grip_msg);
end