function ros_cur_q = mat2rosJoints(mat_cur_q)
%--------------------------------------------------------------------------
% mat2rosJoints
% Set ros joint angle positions from matlab positions [pan, tilt, elbow, w1, w2, w3]. Inverse of
% ros2matlabJoints. Hard-coded here. Need to update to be robust.
% TODO: do I need to include the knuckle here?
%
% Input
% mat_cur_q [1,6]: the six joint angles of the UR5e excluding fingers
%
% Output
% ros_cur_q []: UR5e robot joint angles in this order: shoulder pan, lift, elbow, 3 wrist joint angles.
%--------------------------------------------------------------------------

ros_cur_q = [mat_cur_q(3) mat_cur_q(2) mat_cur_q(1) mat_cur_q(4) mat_cur_q(5) mat_cur_q(6)];
    
    %% Local Variables
    % qi = zeros(1,6);            % array of indeces
    % mat_cur_q = zeros(1,6);     % array of joint angles
    
    
    %% Extract the latest current joint angle values and joint names
     % ros_cur_q = ros_cur_jnt_state_msg.Position;
     % robot_joint_names = ros_cur_jnt_state_msg.Name;
    
     % %% Create a UR5e Dictionary where keys are joint naves and values is the index order
     % ur5e = dictionary(robot_joint_names{1},3,... % elbow
     %                   robot_joint_names{2},7,... % knuckle
     %                   robot_joint_names{3},2,... % lift
     %                   robot_joint_names{4},1,... % pan
     %                   robot_joint_names{5},4,... % w1
     %                   robot_joint_names{6},5,... % w2 
     %                   robot_joint_names{7},6);   % w3
    
    %% Fill joint angles correctly"
    % for i = 1:ur5e.numEntries
    % 
    %     % Create qi as an index by tapping the dictionary with name key and
    %     % getting the index value except for finger
    %     if ~strcmp('robotiq_85_left_knuckle_joint',ros_cur_jnt_state_msg.Name{i})
    %         qi(i) =  ur5e( ros_cur_jnt_state_msg.Name{i} ); % Name is a cell
    % 
    %         % Set the value of ros_cur_q(i) in the appropriate mat_cur_q entry set by qi(i)
    %         ros_cur_q( qi(i) ) = mat_cur_q( i );
    %     end    
    % end

end

