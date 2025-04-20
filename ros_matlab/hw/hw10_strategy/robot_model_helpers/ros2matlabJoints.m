function [mat_cur_q,robot_joint_names] = ros2matlabJoints(ros_cur_jnt_state_msg)
%--------------------------------------------------------------------------
% ros2matlabJoints
% Takes a joint state message, extracts names and joint angles and
% rearranges correctly into matlab format from shoulder pan, lift, elbow, 3
% wrist joint angles.
%
% Input
% ros_cur_jnt_state_msg(sensor_msgs/JointState): names and positions
%
% Output
% mat_cur_q []: UR5e robot joint angles in this order: shoulder pan, lift, elbow, 3 wrist joint angles.
%--------------------------------------------------------------------------
    
    %% Local Variables
    qi = zeros(1,6);            % array of indeces
    mat_cur_q = zeros(1,6);     % array of joint angles
    
    %% Extract the latest current joint angle values and joint names
     ros_cur_q = ros_cur_jnt_state_msg.Position;
     robot_joint_names = ros_cur_jnt_state_msg.Name;
    
     %% Create a UR5e Dictionary where keys are joint naves and values is the index order
     ur5e = dictionary(robot_joint_names{1},3,... % elbow
                       robot_joint_names{2},7,... % knuckle
                       robot_joint_names{3},2,... % lift
                       robot_joint_names{4},1,... % pan
                       robot_joint_names{5},4,... % w1
                       robot_joint_names{6},5,... % w2 
                       robot_joint_names{7},6);   % w3
    
    %% Fill joint angles correctly"
    for i = 1:ur5e.numEntries
    
        % Create qi as an index by tapping the dictionary with name key and
        % getting the index value except for finger
        if ~strcmp('robotiq_85_left_knuckle_joint',ros_cur_jnt_state_msg.Name{i})
            qi(i) =  ur5e( ros_cur_jnt_state_msg.Name{i} ); % Name is a cell
    
            % Set the value of ros_cur_q(i) in the appropriate mat_cur_q entry set by qi(i)
            mat_cur_q( qi(i) ) = ros_cur_q( i );
        end    
    end
end
