function [jointAngles,jointNames] = getJointAngles(this)
    % Receive message in ROS format
    currentJointStatesMsg = receive(this.joint_state_sub,2);

    % Reorder from ROS format to Matlab format, need names.
    % Local Variables
    qi = zeros(1,6);                     % array of indeces
    jointAngles = zeros(1,6);     % array of joint angles

    % Extract the latest current joint angle values and joint names
    ros_cur_q = currentJointStatesMsg.Position;
    jointNames = currentJointStatesMsg.Name;

    % Create a UR5e Dictionary where keys are joint naves and values is the index order
    ur5e = dictionary(jointNames{1},3,... % elbow
                    jointNames{2},7,... % knuckle
                    jointNames{3},2,... % lift
                    jointNames{4},1,... % pan
                    jointNames{5},4,... % w1
                    jointNames{6},5,... % w2
                    jointNames{7},6);   % w3

    % Fill joint angles correctly"
    for i = 1:ur5e.numEntries

        % Create qi as an index by tapping the dictionary with name key and
        % getting the index value except for finger
        if ~strcmp('robotiq_85_left_knuckle_joint',currentJointStatesMsg.Name{i})
            qi(i) =  ur5e( currentJointStatesMsg.Name{i} ); % Name is a cell

            % Set the value of ros_cur_q(i) in the appropriate mat_cur_q entry set by qi(i)
            jointAngles( qi(i) ) = ros_cur_q( i );
        end
    end
end