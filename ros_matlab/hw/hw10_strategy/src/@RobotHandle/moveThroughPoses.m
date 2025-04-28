function result = moveThroughPoses(this, poses, duration)
    %1. Size in terms 4x4xn
    num_traj_points = size(poses, 3);

    % Create trajectory of joint angles as a row matrix (m x 6) where, we have m waypoints by 6 joint angles for the UR5e
    jointAngles = zeros(num_traj_points, 6);

    % Get current joint states to then set the initial guess of the IK solver
    [currentJointAngles, ~] = this.getJointAngles();

    % Get joint angles for each pose in trajectory
    for i = 1:num_traj_points
        poseCon = constraintPoseTarget('tool0', 'TargetTransform', poses(:,:,i), 'Weights', this.gik_weights);

        [thisPoseJointAngles, ~] = this.gik(currentJointAngles, poseCon, this.jointCon);

        % Set 1st row of des_q's to the 1st row of mat_joint_traj
        jointAngles(i,:) = thisPoseJointAngles(1,:);
        currentJointAngles = thisPoseJointAngles(1,:); % Update for next iteration
    end


    result = this.setJointAngles(jointAngles, duration);
end