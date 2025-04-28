function resetWorld(this)
%--------------------------------------------------------------------------
% resetWorld()
% Calls Gazebo service to reset the world
% Input: (dict) optns
% Output: None
%--------------------------------------------------------------------------

    ros_client_msg = rosmessage(this.res_client);

    % Call reset service
    status = waitForServer(this.res_client);
    if status
        try
            logPrint(4, 'resetWorld', 1, '', "Calling world reset service");
            [testresp,status,state] = call(this.res_client, ros_client_msg,"Timeout",3);
            logPrint(4, 'resetWorld', 2, '', "Result State: %s", state);
            logPrint(4, 'resetWorld', 2, '', "Result Status: %s", status);
            logPrint(4, 'resetWorld', 2, '', "Result Message: %s", testresp.MessageType);
            logPrint(4, 'resetWorld', 3, '', "Error Code: %s", string(testresp.ErrorCode));
            logPrint(4, 'resetWorld', 3, '', "ErrorString: %s", string(testresp.ErrorString));
        catch
            logPrint(4, 'resetWorld', 1, 'red', "Reset world service failed");
        end
    end
end