function get_gripper_pose_and_displace(optns)

%% Local variables
frameAdjustmentFlag = 1;     % Indicates matlab's base_link does not match ros. Need adjustment.
toolAdjustmentFlag  = 1;     % Indicates we have fingers but have not adjusted IKs for it.    
tf_listening_time   = 10;

tftree       = rostf('DataFormat','struct');     
base         = 'base_link';
end_effector = 'tool0'; % When finger is properly modeled use 'gripper_tip_link'
    
% Get end-effector pose wrt to base via getTransform(tftree,targetframe,sourceframe), where sourceframe is the reference frame 
% However, that order of parameters does not give the right answer. Need to reverse them!!!
try
    current_pose = getTransform(tftree,base,end_effector,rostime('now'),'Timeout', tf_listening_time);
catch
    % Try again
    current_pose = getTransform(tftree,base,end_effector,rostime('now'),'Timeout', tf_listening_time);
end
    
% Convert gripper pose to matlab format
mat_R_T_G = ros2matlabPose(current_pose, ...
                           optns{'frameAdjustmentFlag'}, ...
                           optns{'toolAdjustmentFlag'},...
                           optns); 

% Move forward 10cm from central position
offset = 0.1;

mat_R_T_G(2,4) = mat_R_T_G(2,4) + offset;
moveTo(mat_R_T_G, optns);

% Move back 10cm from central position
mat_R_T_G(2,4) = mat_R_T_G(2,4) - 2*offset;
moveTo(mat_R_T_G, optns);

% Return to central position
mat_R_T_G(2,4) = mat_R_T_G(2,4) + offset;
moveTo(mat_R_T_G, optns);

% Move Right (facing towards the front of robot, i.e. matlab -x direction)
mat_R_T_G(1,4) = mat_R_T_G(1,4) - offset;
moveTo(mat_R_T_G, optns);

% Move Left
mat_R_T_G(1,4) = mat_R_T_G(1,4) + 2*offset;
moveTo(mat_R_T_G, optns);

% Return to central position
mat_R_T_G(1,4) = mat_R_T_G(1,4) - offset;
moveTo(mat_R_T_G, optns);