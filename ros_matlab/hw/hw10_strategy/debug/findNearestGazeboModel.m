function [modelName, modelPos, dist] = findNearestGazeboModel(pos, optns)
% findNearestGazeboModel  Find the nearest Gazebo model to a given position
%   Inputs:
%     pos    - 1x3 vector [X,Y,Z] specifying the query position
%     optns  - options object for Gazebo interface (passed to getModels)
%   Outputs:
%     modelName - string name of the closest model
%     modelPos  - 1x3 vector position of that model
%     dist       - scalar Euclidean distance from pos to modelPos

    global cachedModelNames;
    global cachedModelPoses;
    global cachedModelPosesLastUpdated;

    nowtime = now();

    if isempty(cachedModelPoses) || isempty(cachedModelPosesLastUpdated) || isempty(cachedModelNames) || (nowtime - cachedModelPosesLastUpdated > 10000)
        cachedModelPosesLastUpdated = nowtime;
        
        % Retrieve all model names
        allModels = getModels(optns);
        cachedModelNames = allModels.ModelNames;  % cell array of char arrays
        cachedModelPoses = cell(size(cachedModelNames, 1), 1);

        for i = 1:size(cachedModelNames, 1)
            % Get the model object by name
            model = get_model_pose(cachedModelNames{i}, optns);
            % Extract position struct
            pStruct = model.Pose.Position;  % struct with fields X, Y, Z
            % Convert to vector
            cachedModelPoses{i} = [pStruct.X, pStruct.Y, pStruct.Z];
        end
    end

    % Initialize outputs
    modelName = '';
    modelPos  = [NaN, NaN, NaN];
    dist       = inf;

    

    % Loop through each model to find its position and distance
    for i = 12:size(cachedModelNames, 1)
        % Convert to vector
        currentPos = cachedModelPoses{i};
        % Compute Euclidean distance
        d = sqrt(sum((currentPos - pos) .^ 2));
        % Update if closer
        if d < dist
            dist       = d;
            modelName = cachedModelNames{i};
            modelPos  = currentPos;
        end
    end
end
