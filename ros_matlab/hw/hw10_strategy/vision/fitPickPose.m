% Start with only cylinders
function [model, topCenterPosition, pickPose] = fitPickPose(objectPtCloud, label)
    % Assume Cylinder and that it's either upright or lying on its side. Nowhere in between.
    
    % Attempt to fit an upright cylinder to the filtered point cloud
    logPrint(3, 'fitPickPose-'+string(label), 5, '', "Attemping to fit a vertical cylinder to point cloud.");
    [model, inIndices, outIndices, meanError] = pcfitcylinder(objectPtCloud, 0.005, [0 0 1]);
    logPrint(4, 'fitPickPose-'+string(label), 6, '', "Radius: %.3f, Height: %.3f, Mean Error: %.3f, Num Inside Pts: %d, Num Outside Pts: %d", model.Radius, model.Height, meanError, size(inIndices,1), size(outIndices,1));
    if inIndices < 0.99*length(objectPtCloud.Location)
        logPrint(4, 'fitPickPose-'+string(label), 6, '', "Vertical Cylinder fit was not good enough. Trying again without the vertical constraint.");
        [model, inIndices, outIndices, meanError] = pcfitcylinder(objectPtCloud, 0.005);
        logPrint(4, 'fitPickPose-'+string(label), 6, '', "New Cylinder: Radius: %.3f, Height: %.3f, Mean Error: %.3f, Num Inside Pts: %d, Num Outside Pts: %d", model.Radius, model.Height, meanError, size(inIndices,1), size(outIndices,1));
    end
    
    if model.Radius > 0.1
        logPrint(3, 'fitPickPose-'+string(label), 6, 'red', "Object does not match cylinder fit. No pick pose will be created.");
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
    logPrint(3, 'fitPickPose-'+string(label), 5, '', "Generating a pick pose at the top center of the object. This even works for lying cylinders.");
    logPrint(3, 'fitPickPose-'+string(label), 6, '', "Center Position: [%.3f, %.3f, %.3f]", model.Center(2), model.Center(1), model.Center(3));
    p1 = model.Parameters(1:3);
    p2 = model.Parameters(4:6);
    % vertAngle = acosd( (p2(3)-p1(3)) / norm(p2 - p1) );
    pickPose = transl(-model.Center(2), model.Center(1), objectPtCloud.ZLimits(1,2)) * trotx(-pi);
    
    logPrint(3, 'fitPickPose-'+string(label), 6, '', "Pick Position: [%.3f, %.3f, %.3f]", model.Center(2), model.Center(1), objectPtCloud.ZLimits(1,2));
    % if vertAngle < 45
    % else
    %     % FIXME: This is a placeholder
    %     pickPose = transl(-model.Center(2), model.Center(1), objectPtCloud.ZLimits(1,2)) * trotx(-pi) * trotz(pi/2);
    % end

    % Get the top center of the cylinder
    topCenterPosition = [model.Center(1), model.Center(2), objectPtCloud.ZLimits(1,2)];
end