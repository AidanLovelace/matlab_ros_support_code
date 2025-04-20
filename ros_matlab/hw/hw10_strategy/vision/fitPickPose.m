% Start with only cylinders
function [model, topCenterPosition, pickPose] = fitPickPose(objectPtCloud, label)
    % Assume Cylinder and that it's either upright or lying on its side. Nowhere in between.
    
    % Attempt to fit an upright cylinder to the filtered point cloud
    [model, inIndices, ~, ~] = pcfitcylinder(objectPtCloud, 0.005, [0 0 1]);
    if inIndices < 0.99*length(objectPtCloud.Location)
        [model, inIndices, ~, ~] = pcfitcylinder(objectPtCloud, 0.005);
    end

    if model.Radius > 0.1
        % If the radius is too large, we assume it's not a cylinder
        % and return an empty model
        model = [];
        topCenterPosition = [];
        pickPose = [];
        return;
    end

    % Find the best pick pose by finding whether the cylinder is upright or lying on its side
    % and then determining the rotation of the gripper to align with the cylinder

    % First, find the angle between the cylinder's axis and the z-axis
    % pickPose = [];
    p1 = model.Parameters(1:3);
    p2 = model.Parameters(4:6);
    % vertAngle = acosd( (p2(3)-p1(3)) / norm(p2 - p1) );
    pickPose = transl(-model.Center(2), model.Center(1), objectPtCloud.ZLimits(1,2)) * trotx(-pi);
    % if vertAngle < 45
    % else
    %     % FIXME: This is a placeholder
    %     pickPose = transl(-model.Center(2), model.Center(1), objectPtCloud.ZLimits(1,2)) * trotx(-pi) * trotz(pi/2);
    % end

    % Get the top center of the cylinder
    topCenterPosition = [model.Center(1), model.Center(2), objectPtCloud.ZLimits(1,2)];
end