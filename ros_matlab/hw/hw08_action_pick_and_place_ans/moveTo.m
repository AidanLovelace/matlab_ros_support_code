function [traj_result,mat_joint_traj] = moveTo(mat_R_T_M,optns)
    %----------------------------------------------------------------------
    % moveTo 
    % Moves to a desired location either as a single point or a trajectory
    % of waypoints.
    % 
    % 01 Set goal or waypoints as a homogeneous transform
    % 02 Convert to joint angles via IKs
    %
    % Expansion/TODO:
    % - Check if robot already at desired position. Then skip action calls.
    %
    % Inputs
    % mat_R_T_M [4x4]: object pose wrt to base_link
    % ops (dictionary): contains options like debug, toolFlag, traj_steps, traj_duration, etc
    %
    % Outputs:
    % traj_result (bool): 0 indicates success, other failure.
    %----------------------------------------------------------------------
    
    
    %% 1. Local variables
    r = optns{'rHandle'};

    %$ Local variables
    traj_duration = optns{'traj_duration'}; 
    
    %% Do not use ctraj, just 1 point.
    mat_traj = mat_R_T_M;
    
    %% 3. Convert to joint angles via IKs
    disp('Converting waypoints to joint angles...');
    [mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(mat_traj,optns);

    %% Visualize trajectory
    if optns{'debug'}
        rate = rateControl(1);
        for i = 1:size(mat_joint_traj,1)
            r.UR5eROBOT.show(mat_joint_traj(i,:),FastUpdate=true,PreservePlot=false);
            rate.waitfor;
        end
    end
    
    %% 4. Create action client, message, populate ROS trajectory goal and send
    % Instantiate the 
    % Create action goal message from client
    traj_goal = rosmessage(r.pick_traj_act_client); 

    %% ROS-Matlab Bug 2025, requires clearing feedback/result
    r.pick_traj_act_client.FeedbackFcn = [];
    r.pick_traj_act_client.ResultFcn = [];
    
    % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');
    traj_goal = convert2ROSPointVec(mat_joint_traj,...
                                    rob_joint_names,...
                                    1,... %traj_steps
                                    traj_duration,...
                                    traj_goal,...
                                    optns);
    
    % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    
    try waitForServer(r.pick_traj_act_client)
        disp('Connected to Arm server. Moving arm...')
        [traj_result,state,status] = sendGoalAndWait(r.pick_traj_act_client,traj_goal);
    catch
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(r.pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

    % If you want to cancel the goal, run this command
    %cancelGoal(pick_traj_act_client);
end