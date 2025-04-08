function traj_goal = convert2ROSPointVec(mat_joint_traj, robot_joint_names, traj_steps, traj_duration, traj_goal, optns)
%--------------------------------------------------------------------------
% convert2ROSPointVec
% Converts all of the matlab joint trajectory values into a vector of ROS
% Trajectory Points. 
% 
% Make sure all messages have the same DataFormat (i.e. struct)
%
% Inputs:
% mat_joint_traj (n x q) - matrix of n trajectory points for q joint values
% robot_joint_names {} - cell of robot joint names
% traj_goal (FollowJointTrajectoryGoal)
% optns (dict) - traj_duration, traj_steps, ros class handle
%
% Outputs
% vector of TrajectoryPoints (1 x n)
%--------------------------------------------------------------------------
    
    % Get robot handle. Will work with r.point
    r = optns{'rHandle'};

    % Compute time step as duration over steps
    timeStep = traj_duration / traj_steps;
    
    % TODO: Set joint names. Note: must remove finger at index 2
    traj_goal.Trajectory.JointNames = robot_joint_names([1,3:7]);
    
  
    %% Set Points

    % Set an array of cells (currently only using 1 pt but can be extended)
    % points = cell(1,traj_steps);

    % TODO: Create Point Message
    pt = rosmessage('trajectory_msgs/JointTrajectoryPoint', 'DataFormat','struct');
    
    % TODO: Fill r.point: extract each waypoint and set it as a 6x1 (use transpose)
    mat_joint_traj = [mat_joint_traj(3) mat_joint_traj(2) mat_joint_traj(1) mat_joint_traj(4) mat_joint_traj(5) mat_joint_traj(6)];
    pt.Positions     = mat_joint_traj';      % set it as a 6x1
    pt.TimeFromStart = rosduration(timeStep,'DataFormat','struct');
    r.point = pt;

    % TODO: Set inside points cell
    % points.Positions       = mat_joint_traj;

    % TODO: Copy points to traj_goal.Trajectory.Points
    traj_goal.Trajectory.Points = pt;
    
end