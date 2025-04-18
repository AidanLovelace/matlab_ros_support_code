function resetWorld(optns)
%-------------------------------------------------------------------------- 
% resetWorld()
% Calls Gazebo service to reset the world
% Input: (dict) optns
% Output: None
%-------------------------------------------------------------------------- 
    if optns{'debug'}
        disp('Resetting the world...');
    end    
    
    % TODO: 01 Get robot handle
    r = optns{"rHandle"};
    
    % TODO: 02 Create Empty Simulation message
    reset_message = rosmessage(r.res_client);
    
    % TODO: 03 Call reset service
    call(r.res_client,reset_message,"Timeout",3);

end