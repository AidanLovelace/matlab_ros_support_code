function result = moveToPose(this, pose, duration, traj_steps)

    if nargin < 4
        traj_steps = 5;
    end

    if nargin < 3
        duration = 0.5;
    end

    % poses = ctraj(this.currentPose, pose, lspb(0, 1, traj_steps));

    result = this.moveThroughPoses([ pose ], duration);
end