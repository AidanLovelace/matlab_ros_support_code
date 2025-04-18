function optns = initRobotConn(masterhostIP, nodeIP, robotName)
    %----------------------------------------------------------------------
    % initRobotConn
    % Executes startup code before resesting the world and moving the arm.
    %-----------------------------------------------------------------------
        % Init Params
        if nargin == 0
            masterhostIP = "100.113.159.11";
            nodeIP = "100.127.95.110";
            robotName = "UR5e";
        end

        % Check if ROS Global Node is running
        cprintf('blue','Establishing Connection to ROS Master Node...\n');
        if ros.internal.Global.isNodeActive() == 1
            % ROS Global Node is running
            % Get the node handle
            node = ros.internal.Global.getNodeHandle();

            if contains(node.MasterURI, masterhostIP)
                cprintf('text', ' - ROS Global Node already running on: %s\n', node.MasterURI);
            else
                cprintf('text', ' - ROS Global Node was already running on: %s\n', node.MasterURI);
                cprintf('text', ' - Restarting global node to run on %s...\n', masterhostIP);
                rosshutdown;
                rosinit(masterhostIP, 11311, "NodeHost", nodeIP);
            end
        else
            cprintf('text', ' - Starting ROS Global Node on %s...\n', masterhostIP);
            rosinit(masterhostIP, 11311, "NodeHost", nodeIP);
        end
        cprintf('text', ' - ROS Global Node ready!\n');
        
        % Robot Class Handle
        if robotName == "UR5e"
             r = rosClassHandle_UR5e;
        else
            disp('Unknown handle name. Exting...');
            exit();
        end
        
       %% Global dictionary with all options
       keys   = ["debug", ...
                "toolFlag",...
                "traj_steps", ...
                "x_offset", ...
                "y_offset", ...
                "z_offset", ...
                'gripPos', ...
                "traj_duration",...
                "frameAdjustmentFlag", ...
                "toolAdjustmentFlag", ...
                "toolAdjustment", ...
                "tf_listening_time",...
                "rHandle",...
                "cleanStart"];
       
       values = {  0,...        "debug"
                   0,...        "toolFlag"
                   1,...        "traj_steps"
                   0,...        "x_offset"
                   0,...        "y_offset"
                   0.2,...      "z_offset"
                   0.23,...     'gripPos'
                   1,...        "traj_duration"
                   1,...        "frameAdjustmentFlag"
                   1,...        "toolAdjustmentFlag"
                   0.165,...    "toolAdjustment"
                   10,...       "tf_listening_time"
                   r,...        "rHandle"
                   true,...     "cleanStart"
                   };
        optns = dictionary(keys,values);    
    end