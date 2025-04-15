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
    %
    % Note:
    % When new objects are added to the scene, new pick strategies will
    % need to be added to this file. That will require tinkering.
    %----------------------------------------------------------------------
     
    % Check type of objectData. If matrix, assign directly; otherwise
    % extract.
    if isequal(size(objectData), [4, 4]) && isnumeric(objectData)
        mat_R_T_M = objectData;
        label = "can";
    
    % Extract pose and label from object
    else
        label = objectData{1,1};         
        mat_R_T_M = objectData{1,2};         
    end
    pause(2);

    %% 1) Determine z offset and grip distance required
    %   z offset includes offset for both the base and the gripper
        if strcmpi(string(label), "pouch")
            zOffset = 0.156;
            doGripValue = 0.59; %0.61
        
        elseif strcmpi(string(label), "vCan")  %%% COMPLETE
            zOffset = 0.17;
            doGripValue = 0.23;
        
        elseif strcmpi(string(label), "hCan") % COMPLETE
            zOffset = 0.15;
            doGripValue = 0.231;
        
        elseif strcmpi(string(label), "vBottle") %% COMP
            zOffset = 0.24;
            doGripValue = 0.5167;        
        
        elseif strcmpi(string(label), "hBottle") % COMPLETE
            zOffset = 0.12;
            doGripValue = 0.21;

        elseif strcmpi(string(label), "marker")
            zOffset = -1; % TODO
            doGripValue = -1; % TODO
        
        elseif strcmpi(string(label), "spam")
            zOffset = -1; % TODO
            doGripValue = -1; % TODO
        end

    %% 2) Move to desired location
        % Account for base offset + Hover over object
        if strcmp(strategy,'topdown')
            if strcmpi(string(label), "pouch")
                over_R_T_M = lift(mat_R_T_M,0.25);
                MoveToHover = moveTo(over_R_T_M,optns);
    
                mat_R_T_M(3,4)= (mat_R_T_M(3,4)+zOffset);
                mat_R_T_M = mat_R_T_M * trotz(pi/8);
                MoveToOb = moveTo(mat_R_T_M, optns);
            else 
                over_R_T_M = lift(mat_R_T_M,0.25);
                MoveToHover = moveTo(over_R_T_M,optns);
    
                mat_R_T_M(3,4)= mat_R_T_M(3,4)+zOffset;
                MoveToOb = moveTo(mat_R_T_M, optns);
            end       

        elseif strcmpi(strategy,'direct')
            traj_result = moveTo(mat_R_T_M,optns);
        end
        % Grip object
        [grip_result,grip_state] = doGrip('pick',optns,doGripValue); 
        grip_result = grip_result.ErrorCode;
        pause(9);
end