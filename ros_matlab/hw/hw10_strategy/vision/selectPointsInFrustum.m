function subsetCloud = selectPointsInFrustum(ptCloud, frustumCornersWorld)
    % selectPointsInFrustum Extract points of a ptCloud within the specified frustum
    %   subsetCloud = selectPointsInFrustum(ptCloud, frustumCornersWorld)
    % Inputs:
    %   ptCloud               - pointCloud object in world coordinates
    %   frustumCornersWorld   - 8x3 frustum corners as [nearTL; nearTR; nearBR; nearBL; farTL; farTR; farBR; farBL]
    %
    % Output:
    %   subsetCloud           - pointCloud containing only points inside the frustum
    
    % Define faces of frustum (each row indices into corners)
    faces = [1 2 3; 1 3 4; 5 6 7; 5 7 8; 1 2 6; 2 3 7; 3 4 8; 4 1 5];
    corners = frustumCornersWorld;
    
    % Compute centroid to orient normals inward
    centroid = mean(corners,1);
    
    % Preallocate plane coefficients (8 planes, 4 coeffs each)
    numFaces = size(faces,1);
    planes = zeros(numFaces,4);
    
    % Build each plane: [n_x n_y n_z d]
    for f = 1:numFaces
        idx = faces(f,:);
        p1 = corners(idx(1),:);
        p2 = corners(idx(2),:);
        p3 = corners(idx(3),:);
        n = cross(p2-p1, p3-p1);    % 1x3 row vector
        n = n / norm(n);
        d = -dot(n,p1);
        % flip to point inward
        if dot(n,centroid) + d < 0
            n = -n; d = -d;
        end
        planes(f,:) = [n d];    % use row vector n
    end
    
    % Flatten point cloud locations
    locs = ptCloud.Location;
    flat = reshape(locs, [], 3);
    
    % Test all points against all half-spaces
    maskAll = true(size(flat,1),1);
    for f = 1:numFaces
        n = planes(f,1:3);
        d = planes(f,4);
        maskAll = maskAll & (flat * n' + d >= 0);
    end
    
    % Select and return subset via built-in select
    indices = find(maskAll);
    subsetCloud = select(ptCloud, indices);
end