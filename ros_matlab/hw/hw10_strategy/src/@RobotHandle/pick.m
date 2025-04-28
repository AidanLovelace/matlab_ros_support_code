function grip_result = pick(this, object)
    %----------------------------------------------------------------------
    % pick
    % Top-level function to executed a complete pick.
    %
    % 01 Calls moveTo to move to desired pose
    % 02 Calls doGrip to execute a grip
    %
    % Inputs
    % mat_R_T_M [4x4]: object pose wrt to base_link
    % mat_R_T_G  [4x4]: gripper pose wrt to base_link used as starting point in ctraj (optional)
    % optns (dict): options
    %
    % Outputs:
    % ret (bool): 0 indicates success, other failure.
    %
    % Note:
    % When new objects are added to the scene, new pick strategies will
    % need to be added to this file. That will require tinkering.
    %----------------------------------------------------------------------

    mat_R_T_M = object.pickPose;

    label = object.label;

    travelZOffset = 0.2;
    hoverZOffset = 0.06;
    zOffset = 0;
    doGripValue = 0;
    % 1) Determine z offset and grip distance required
    %   z offset includes offset for both the base and the gripper
    if contains(string(label), "pouch")
        zOffset = 0.00;
        doGripValue = 0.625; %0.61

    elseif contains(string(label), "vCan")
        zOffset = 0.005;
        doGripValue = 0.229;

    elseif contains(string(label), "hCan")
        zOffset = -0.015;
        doGripValue = 0.231;

    elseif contains(string(label), "vBottle")
        zOffset = 0.01;
        doGripValue = 0.5167;
        % doGripValue = 0.215;

    elseif contains(string(label), "hBottle")
        zOffset = -0.015;
        doGripValue = 0.215;

    elseif contains(string(label), "marker")
        zOffset = 0;
        doGripValue = 0.7;

    elseif contains(string(label), "spam")
        zOffset = 0;
        doGripValue = 0.3;
    end
    logPrint(4, 'pick-'+string(label), 3, '', "travelZOffset = %.2f", travelZOffset);
    logPrint(4, 'pick-'+string(label), 3, '', "hoverZOffset = %.2f", hoverZOffset);
    logPrint(4, 'pick-'+string(label), 3, '', "zOffset = %.2f", zOffset);
    logPrint(4, 'pick-'+string(label), 3, '', "doGripValue = %.2f", doGripValue);

    % 2) Move to desired location
    % Account for base offset + Hover over object
    travel_over_R_T_M = lift(mat_R_T_M, travelZOffset);
    over_R_T_M = lift(mat_R_T_M, hoverZOffset);
    mat_R_T_M = lift(mat_R_T_M, zOffset);
    logPrint(4, 'pick-'+string(label), 3, '', "Moving above object");
    this.moveToPose(travel_over_R_T_M, 0.75);
    % pause(0.25);
    logPrint(4, 'pick-'+string(label), 3, '', "Moving directly over object");
    this.moveToPose(over_R_T_M, 0.75);
    % pause(0.25);
    logPrint(4, 'pick-'+string(label), 3, '', "Moving to final pick position");
    this.moveToPose(mat_R_T_M, 0.75);
    this.moveToPose(mat_R_T_M, 0.75);
    % pause(0.25);

    logPrint(4, 'pick-'+string(label), 3, '', "Closing gripper");
    [grip_result, ~] = this.setGripper(doGripValue);
    grip_result = grip_result.ErrorCode;
    pause(2);
    logPrint(4, 'pick-'+string(label), 3, '', "Moving back to directly over object");
    this.moveToPose(travel_over_R_T_M, 1);
    pause(0.5);
end