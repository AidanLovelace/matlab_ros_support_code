function grip_result = moveToBin(strategy,mat_current,mat_bin,optns)
%----------------------------------------------------------------------
% moveToBin 
% Top-level function to execute a complete place. 
% 
% 01 Move according to strategy
% 02 Open fingers
%
% Inputs
% strategy (string):    direct or topdown
% mat_current [4x4]:    object pose wrt to base_link
% mat_bin  [4x4]:       bin pose wrt to base_link
%
% Outputs:
% ret (bool): 0 indicates success, other failure.
%----------------------------------------------------------------------

    %% 1. Vars / Dictionary of options
    % z_offset = optns("z_offset");
    % z_offset = z_offset{1};
    
    %% 2. Move to desired location
    if strcmp(strategy,'topdown') % Lift and then go to bin       
        
        % Lift
        disp('Lifting object....')
        over_R_T_M = lift(mat_current, 0.20);
        traj_result = moveTo(over_R_T_M, optns);

        % Displace
        if ~traj_result
            disp('Moving to bin...');
            traj_result = moveTo(mat_bin, optns);
        end
        
    % Move directly
    elseif strcmpi(strategy,'direct')
        traj_result = moveTo(mat_bin, optns);
    end


    %% 3. Place if successfull (check structure of resultState). Otherwise...
    if ~traj_result
        [grip_result, ~] = doGrip('place',optns); 
        grip_result = grip_result.ErrorCode;
    end
end