function [mat_joint_traj,robot_joint_names] = ikWithLimits(mat_traj,optns)
    %-------------------------------------------------------------
    % convertPoseTraj2JointTraj
    % Compute IKs for Cartesian trajectory. Will need to:
    % 1. Create an array of joint angles the same size as trajectory
    % 2. Extract a UR5 object from robot class w/ needed adjustments for URDF Fwd Kins
    % 4. Extract numerical IK object for Robot from robot class
    % 5. Loop through trajectory
    % 6. Extract solution w/ checks-- first one for right/elbow/down. 
    %
    % Input:
    % - mat_traj [] - array of 4 x 4 x n waypoints of homogeneous trasformation trajectories
    %
    % Output:
    % - mat_joint_traj [] - array of n x 6 waypoints of joint angle trajectories
    % - robot_joint_names {} - array of cells of joint names
    %-------------------------------------------------------------
    
    % Extract robot
    r = optns{'rHandle'};

    %1. Size in terms 4x4xn
    traj_sz = size(mat_traj);

    % 1 traj point: just 4x4 but not 4x4xn
    if length(traj_sz) == 2 
        num_traj_points = 1;

    % ctraj or similar called
    else
        num_traj_points = traj_sz(1,3);
    end
    
    % 2 Create trajectory of joint angles as a row matrix (m x 6) where, we have m waypoints by 6 joint angles for the UR5e
    mat_joint_traj = zeros(num_traj_points,6);    
   
    % Get current joint states to then set the initial guess of the IK solver
    [mat_cur_q,robot_joint_names] = get_current_joint_states(optns);

    % Check Joint Angle Integrity for all angles
    % if ~checkJointLimits(mat_cur_q)
    %     error('Subscribed joints outside joint limits.')
    %     fprintf('mat_cur_q: '); fprintf('%.2f,', mat_cur_q);fprintf('\n')
    % end
    
    % Elbow Down a problem? Force elbow and w4 guesses:
    % mat_cur_q(3) = pi/2; mat_cur_q(4) = -pi/2;

    % 5. Go through trajectory loop
    % Check for time complexity. Can we improve efficiency.
    for i = 1:num_traj_points
        poseCon = constraintPoseTarget('tool0', 'TargetTransform', mat_traj(:,:,i), 'Weights', r.gik_weights);
        [des_q, ~] = r.gik(mat_cur_q, poseCon, r.jointCon); 

        % Check Joint Angle Integrity
        % if max( abs(des_q) ) > 2*pi || ~checkJointLimits(des_q)
        %     disp('IK joints outside joint limits. Will keep the same joint angles...')
        %     mat_joint_traj(i,:) = mat_cur_q;
        %     return 
        % end


        % Print soln for debugging purposes:
        if optns{'debug'}
            fprintf('IKs: '); fprintf('%.2f,', des_q);fprintf('\n')
        end
        
        % 6. Set 1st row of des_q's to the 1st row of mat_joint_traj
        mat_joint_traj(i,:) = des_q(1,:);         %q_sz = size(des_q);

        % 7. Update joint angles for next round: mat_cur_q
        [mat_cur_q,~] = get_current_joint_states(optns);
    end
% Debug disp(mat_joint_traj)
end
