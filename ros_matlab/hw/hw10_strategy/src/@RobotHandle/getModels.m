function models = getModels(this)
%--------------------------------------------------------------------------
% getModels
% This method will create an action client that talks to Gazebo's
% get_world_properties to get all models.
%
% Inputs: (dict) optns
% Output: (gazebo_msgs/GetWorldPropertiesResponse cell): models
%--------------------------------------------------------------------------

    % 01 Create model_client_msg
    get_models_client_msg = rosmessage(this.get_models_client);

    % 03 Call client
    status = waitForServer(this.get_models_client);
    if status
        try
            [models,~,~] = call(this.get_models_client,get_models_client_msg);
        catch
            disp('Error - models not listed...')

        end
    end

end