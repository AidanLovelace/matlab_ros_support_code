function traj_goal = ros2matPackTraj(mat_joint_traj,names,traj_steps,traj_duration,traj_goal)
%---
% Converts a matlab joint trajectory (a matrix of joint row values) into a 
% cell of ROS trajectory_msgs/JointTrajectoryPoint. We need cells to
% capture a more complicated object on the matlab side {}
% 
% After that, we take the vector of points and populate the trajectory_msgs/JointTrajectory
%%
% Inputs: 
% mat_joint_traj - n x q matrix with n trajectory entries of q joint values
% names {}       - cell of joint names for the robot
% traj_steps     - number of steps in trajectory
% traj_duration  - duration of trajectory in seconds
% traj_goal - ROS message
%       MessageType: 'trajectory_msgs/JointTrajectory'
%       Header:     [1×1 struct]
%       JointNames: {0×1 cell}
%       Points:     [0×1 struct]
% Update goal 

    %% A. Convert matlab joint trajectory to vector of ROS Trajectory Points
    time = 0;
    increment = traj_duration / traj_steps;

    % Create a cell of TrajectoryPoints
    [~,~,d] = size(mat_joint_traj);
    cellTrajPts = cell(1,d);

    for i = 1:length(d)
        
        % Set the appropriate row of joint values in the trajectory to the ith entry in the cell of ROS type TrajectoryPoint's Positions
        cellTrajPts{i}.Positions = mat_joint_traj(i,:);

        time = time + increment;
        cellTrajPts{i}.TimeFromStart = time;

    end

    %% B. Now populate the JointTrajectory:
    traj_goal.JointNames = names;
 
    % Set points equal to vector of points by slicing cell inside vector
    traj_goal.Points = [ cellTrajPts{1:traj_steps} ]; 

    % Finally set the stamp and return
    traj_goal.Header.Stamp = rostime('now','DataFormat','struct');

end