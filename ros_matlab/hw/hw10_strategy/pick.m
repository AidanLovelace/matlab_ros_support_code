function grip_result = pick(strategy,objectData,optns)
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
    %----------------------------------------------------------------------
     
    % Check type of objectData. If matrix, assign directly; otherwise
    % extract.
    if isequal(size(objectData), [4, 4]) && isnumeric(objectData)
        mat_R_T_M = objectData;
        label = "can";
    
    % Extract pose and label from object
    else
        label = objectData(1,1); 
        label = label{1};
        mat_R_T_M = objectData(1,2); 
        mat_R_T_M = mat_R_T_M{1};
    end

    travelZOffset = 0.5;

    %% 1) Determine z offset and grip distance required
    %   z offset includes offset for both the base and the gripper
        if strcmp(string(label), "pouch")
            zOffset = 0.145;
            doGripValue = 0.62;
        elseif strcmp(string(label), "can")
            zOffset = 0.1;
            doGripValue = 0.23;
        elseif strcmp(string(label), "bottle")
            zOffset = 0.12;
            doGripValue = 0.36;
        else
            zOffset = 0.2;
            doGripValue = 0.36;
        end
        zOffset
        doGripValue

    %% 2) Move to desired location
        % Account for base offset + Hover over object
        if strcmp(strategy,'topdown')
            safe_over_R_T_M = lift(mat_R_T_M,travelZOffset);
            over_R_T_M = lift(mat_R_T_M,zOffset);
            MoveSafeToOb = moveTo(safe_over_R_T_M, optns);
            MoveToOb = moveTo(over_R_T_M, optns);
            disp(over_R_T_M);
        
        elseif strcmpi(strategy,'direct')
            traj_result = moveTo(mat_R_T_M,optns);
        end
        % Grip object
        [grip_result,grip_state] = doGrip('pick',optns,doGripValue); 
        grip_result = grip_result.ErrorCode;
end