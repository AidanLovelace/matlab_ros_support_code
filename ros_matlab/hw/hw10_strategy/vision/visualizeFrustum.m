function visualizeFrustum(frustumCornersWorld)
    % visualizeFrustum Display a 3D frustum given its world corners
    %   visualizeFrustum(frustumCornersWorld)
    % Input:
    %   frustumCornersWorld - 8x3xN array as returned by computeFrustumFromBBox
    %
    % The order of rows is: [nearTL; nearTR; nearBR; nearBL; farTL; farTR; farBR; farBL]
    
    for i = 1:size(frustumCornersWorld,3)
        % Split into near and far planes
        nearPts = frustumCornersWorld(1:4, :, i);
        farPts  = frustumCornersWorld(5:8, :, i);
        
        % Draw edges of near and far rectangles
        for j = 1:4
            ni = nearPts(j, :);
            fi = farPts(j, :);
            nj = nearPts(mod(j,4)+1, :);
            fj = farPts(mod(j,4)+1, :);
            % Near edge
            plot3([ni(1), nj(1)], [ni(2), nj(2)], [ni(3), nj(3)], 'r-');
            % Far edge
            plot3([fi(1), fj(1)], [fi(2), fj(2)], [fi(3), fj(3)], 'r-');
            % Side edges
            plot3([ni(1), fi(1)], [ni(2), fi(2)], [ni(3), fi(3)], 'r-');
        end
        hold on;
    end
end