function [cur_pose, cur_jConfig] = moveToCenter(gripper_pose,target_pose,optns)

    %% Compute offset
    g = SE3(gripper_pose);
    t = SE3(target_pose);

    %% Compute position difference starting with target, use .t method in SE3 class
    offset = t.t-g.t; 
        
    %% Add offset in the x-y plane only, keep the same z.
    offset(3) = 0;
    g.t = g.t + offset;
    
    %% Move to new location
    [~,cur_jConfig]=moveTo(g.T, optns);

    %% Get latest pose
    cur_pose = get_gripper_pose(optns);