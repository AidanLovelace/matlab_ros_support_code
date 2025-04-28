function pose = currentPose(this)
    % Return the gripper pose.
    % Since we are computing the pose wrt to tool0, we do not have information
    % of the fingers. The 'fing_alignment' will tell whether the fingers are
    % aligned about the x-axis of the y-axis wrt matlab_coordinates for the
    % base_link. By default x-axis.

    % Get gripper transform
    base         = 'base_link';
    end_effector = 'tool0';                         % When finger is properly modeled use 'gripper_tip_link'

    % Get end-effector pose wrt to base via getTransform(tftree,targetframe,sourceframe), where sourceframe is the reference frame
    % However, that order of parameters does not give the right answer. Need to reverse them!!!
    current_pose = getTransform(this.tftree, base, end_effector, rostime('now'), 'Timeout', this.TFListeningTime);

    % Convert gripper pose to matlab format
    pose = ros2matlabPose(current_pose, true, true, this.options);

    % Adjust orientation of gripper
    ori = eul2tform([0,0,pi]);

    pose(1:3,1:3) = ori(1:3,1:3);
end