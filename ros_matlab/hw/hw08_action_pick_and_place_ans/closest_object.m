function [min_idx,label,min_norm,gripper_pose,obj_pose] = closest_object(objectData,optns)

% Given a pose or structure with pose/label info, compute the norm to one
% of several objects and return the pose/norm/idx/label of closes object.
% 
% Output:
% min_idx: double   - index value corresponding to smallest norm in object
%                     data 
% label:    string  - label of object with min norm
% min_norm: double  - distance from robot gripper to object
% 

    %% Check type of input data
    is_matrix = 0;

    % It's a matrix
    if isequal(size(objectData), [4, 4]) && isnumeric(objectData)
        len = 1; 
        is_matrix = 1;

        % Create an SE3 object to facilitate position extraction
        obj_mat = SE3(objectData);
        
        % Extract the t component
        object_t = obj_mat.t;    
    
        % It's a structure
    else
        len = length(objectData);
        is_matrix = 0;
    end

    % Compute a vector of norms
    norm_vec = zeros(len,1);

    %% Get the robot gripper pose and then current position
    gripper_pose = get_gripper_pose(optns);
    gripper_mat = SE3(gripper_pose);
    gripper_t = gripper_mat.t;

    %% Compute the norm
    if is_matrix && len == 1
        norm_vec(i) = norm(object_t - gripper_t);
    
    % Structure
    else
        for i=1:len
            pose_cell = objectData(i,2);
            obj_mat = pose_cell{1};
            obj_mat = SE3(obj_mat);
            object_t = obj_mat.t;

            norm_vec(i) = norm(object_t - gripper_t);
        end        
    end

    %% Return info for shortest norm
    [min_norm, min_idx] = min(norm_vec);

    %% Label
    label_cell = objectData(min_idx,1);
    label = label_cell{1};

    %% Pose
    pose_cell = objectData(min_idx,2);
    obj_pose = pose_cell{1};

end

