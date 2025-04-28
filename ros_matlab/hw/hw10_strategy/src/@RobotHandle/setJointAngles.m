function result = setJointAngles(this, jointAngles, duration)
    % Convert to ROS waypoint
    timeStep = duration / size(jointAngles, 1);

    currentJointStatesMsg = receive(this.joint_state_sub, 1);

    jointTrajectoryMsg = rosmessage(this.jointControllerClient);

    % When we set the names, remove finger entry 2
    jointTrajectoryMsg.Trajectory.JointNames = currentJointStatesMsg.Name([1,3:7]);

    % Create array to store each waypoint
    points = cell(1,size(jointAngles, 1));

    for idx = 1:size(jointTrajectoryMsg.Trajectory.JointNames, 1)
        jointTrajectoryMsg.GoalTolerance(idx) = rosmessage('control_msgs/JointTolerance', 'DataFormat','struct');
        jointTrajectoryMsg.GoalTolerance(idx).Name = jointTrajectoryMsg.Trajectory.JointNames{idx};
        jointTrajectoryMsg.GoalTolerance(idx).Position = 0;
        jointTrajectoryMsg.GoalTolerance(idx).Velocity = 0;
        jointTrajectoryMsg.GoalTolerance(idx).Acceleration = 0;
    end

    for idx = 1:size(jointTrajectoryMsg.Trajectory.JointNames, 1)
        jointTrajectoryMsg.PathTolerance(idx) = rosmessage('control_msgs/JointTolerance', 'DataFormat','struct');
        jointTrajectoryMsg.PathTolerance(idx).Name = jointTrajectoryMsg.Trajectory.JointNames{idx};
        jointTrajectoryMsg.PathTolerance(idx).Position = 1;
        jointTrajectoryMsg.PathTolerance(idx).Velocity = 10;
        jointTrajectoryMsg.PathTolerance(idx).Acceleration = 10;
    end

    jointTrajectoryMsg.GoalTimeTolerance = rosduration(timeStep, 'DataFormat', 'struct');


    % Set time with format as structure
    for i = 1:size(jointAngles, 1)
        % Extract each waypoint and set it as a 6x1 (use transpose)
	    this.point.Positions     = [jointAngles(i, 3) jointAngles(i, 2) jointAngles(i, 1) jointAngles(i, 4) jointAngles(i, 5) jointAngles(i, 6)]';
	    % this.point.Velocities    = [1; 2; 3; 3; 2; 1];
        this.point.TimeFromStart = rosduration(i*timeStep,'DataFormat','struct');

        % Set inside points cell
        points{i} = this.point;
    end

    jointTrajectoryMsg.Trajectory.Points       = [ points{:} ];

    % Send ros trajectory with traj_steps
    logPrint(8, 'setJointAngles', 1, '', "Sending trajectory to action server.");

    if waitForServer(this.jointControllerClient)
        [res,state,status] = sendGoalAndWait(this.jointControllerClient, jointTrajectoryMsg);
    else
        t = 0;
        [res,state,status] = sendGoalAndWait(this.jointControllerClient, jointTrajectoryMsg);
    end

    logPrint(8, 'setJointAngles', 2, '', "Result State: %s", state);
    logPrint(8, 'setJointAngles', 2, '', "Result Status: %s", status);
    logPrint(8, 'setJointAngles', 2, '', "Result Message: %s", res.MessageType);
    logPrint(8, 'setJointAngles', 3, '', "SUCCESSFUL: %s", string(res.SUCCESSFUL));
    logPrint(8, 'setJointAngles', 3, '', "Error Code: %s", string(res.ErrorCode));
    logPrint(8, 'setJointAngles', 3, '', "ErrorString: %s", string(res.ErrorString));

    % Extract result
    result = res.ErrorCode;
end