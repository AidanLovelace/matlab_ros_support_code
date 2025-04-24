% ObjectIdentifier Class for detecting and identifying objects in robotic vision tasks
%
% This class manages object detection using parallel processing and YOLO detectors,
% handling image capture, point cloud retrieval, and object labeling.
classdef ObjectIdentifier

    properties
        imageDetectors
        robotHandle
        optns
        pool
    end

    methods
        % Constructor for ObjectIdentifier
        %
        % Initializes the ObjectIdentifier with robot options, sets up parallel processing,
        % loads multiple object detection models, and prepares image detectors.
        %
        % Parameters:
        %   optns - Struct containing robot configuration and handle
        %
        % Loads detectors for general, can, and pouch objects and initializes
        % a parallel processing pool for efficient image detection.
        function this = ObjectIdentifier(optns)
            this.robotHandle = optns{'rHandle'};
            this.optns = optns;
            try
                pool = parpool('Processes', 4);
            catch
                pool = gcp('nocreate');
            end

            % Load the detectors for the parallel processing pool
            this.imageDetectors = parallel.pool.Constant(@() { ...
                                                                  load("../../vision_tutorials/detectors/detector_gral_sim.mat").detector, ...
                                                                  load("../../vision_tutorials/detectors/detector_can_sim.mat").detector, ...
                                                                  load("../../vision_tutorials/detectors/detector_pouch_sim.mat").detector ...
                                                              });

            % Calling once to get the image detectors loaded into memory in the pool
            this.getLabeledImageAsync();
        end

        % Identifies and processes objects in a specified inspection zone using YOLO detection and point cloud analysis
        %
        % Captures an image, retrieves point cloud data, performs object detection,
        % and generates detailed object information for potential robotic manipulation.
        %
        % Parameters:
        %   zoneInspect - String identifying the inspection zone to analyze
        %
        % Returns:
        %   objects     - Cell array of detected objects with detailed metadata
        %   ptCloud     - Point cloud data for the entire inspection zone
        %
        % The method performs the following key steps:
        %   1. Asynchronous image capture with YOLO detection
        %   2. Point cloud retrieval and transformation
        %   3. Object detection and frustum computation
        %   4. Individual object point cloud filtering
        %   5. Pick pose estimation
        %   6. Optional Gazebo model comparison
        function [objects, ptCloud] = identifyObjectsHere(this, zoneInspect)
            % Take a picture
            [futureImageDetection, imgMsg, ~] = this.getLabeledImageAsync();
            logPrint(1, 'identifyObjectsHere-'+string(zoneInspect), 1, '', "Picture taken. Processing in background.");

            % Get the point cloud
            logPrint(1, 'identifyObjectsHere-'+string(zoneInspect), 1, '', "Start merged point cloud retrieval.");
            [~, ptCloud, ~] = this.getSeveralPointClouds([]);
            logPrint(1, 'identifyObjectsHere-'+string(zoneInspect), 1, '', "Merged point cloud retrieved.");


            logPrint(4, 'identifyObjectsHere-'+string(zoneInspect), 2, '', "Waiting for the YOLO detection to finish.");
            [bboxes, labels, ~, numOfObjects, ~] = fetchOutputs(futureImageDetection);
            % Get transforms to points of interest
            W_T_R = get_model_pose('robot', this.optns);

            R_T_C = getTransform(this.robotHandle.tftree, ...
                                'base_link', ...
                                imgMsg.Header.FrameId, ...
                                imgMsg.Header.Stamp, ...
                                'Timeout', this.robotHandle.tf_listening_time);

            W_T_ptCloud = pctransform(ptCloud,rigidtform3d(ros2MLPose(W_T_R)));

            % Get the camera info
            logPrint(2, 'identifyObjectsHere-'+string(zoneInspect), 1, '', "Matching bounding boxes from YOLO model to point cloud to get point clouds of individual objects.");
            camInfo = receive(this.robotHandle.caminfo_sub);
            logPrint(4, 'identifyObjectsHere-'+string(zoneInspect), 2, '', "Received camera info (including intrinsics) from camera.");
            camPose = ros2MLPose(W_T_R) * ros2MLPose(R_T_C);

            logPrint(4, 'identifyObjectsHere-'+string(zoneInspect), 2, '', "YOLO detection finished.");

            % Showing detection results
            logPrint(4, 'identifyObjectsHere-'+string(zoneInspect), 3, '', "Number Objects: %d", numOfObjects);
            labelsStr = sprintf(', "%s"', labels);
            logPrint(4, 'identifyObjectsHere-'+string(zoneInspect), 3, '', "Labels: [%s]", labelsStr(2:end));

            frustums = this.bboxToFrustum(camInfo, rigidtform3d(camPose), bboxes, 0.15, 0.75);

            logPrint(4, 'identifyObjectsHere-'+string(zoneInspect), 2, '', "Frustums of bounding boxes computed.");

            logPrint(2, 'identifyObjectsHere-'+string(zoneInspect), 2, '', "Processing each object.");

            % Process each detected object
            objects = cell(numOfObjects, 1);


            for i = 1:numOfObjects
                logPrint(4, 'identifyObjectsHere-'+string(zoneInspect), 3, 'magenta', "Object %d: %s", i, labels(i));

                % Filter point cloud to only include points within the frustum of this bounding box
                objectPtCloud = this.filterPointCloudByBBox(W_T_ptCloud, frustums(:, :, i));

                label = string(labels(i));

                logPrint(4, 'identifyObjectsHere-'+string(zoneInspect), 4, '', "Filtered Point Cloud, %d points.", size(objectPtCloud.Location, 1));
                logPrint(4, 'identifyObjectsHere-'+string(zoneInspect), 4, '', "Attempting to fit a pick pose.");

                [model, topCenterPosition, pickPose, label] = this.fitPickPoseToObject(objectPtCloud, label, bboxes(i, :));

                if isempty(model)
                    logPrint(4, 'identifyObjectsHere-'+string(zoneInspect), 4, 'red', "Pick pose fit failed. This object will not be picked.");
                    continue;
                end

                if this.optns{'comparePicksWithGazebo'}

                    [gazeboModelName, gazeboModelPos, gazeboModelDist] = this.debugGetNearestModelFromGazebo(topCenterPosition);

                    if gazeboModelDist < 0.1
                        logPrint(4, 'identifyObjectsHere-'+string(zoneInspect), 4, '*green', "Matched object to gazebo model. Model Name: %s, ID'd Position: [%.2f, %.2f, %.2f], Model Position: [%.2f, %.2f, %.2f], Distance: %.2f", gazeboModelName, topCenterPosition(1), topCenterPosition(2), topCenterPosition(3), gazeboModelPos(1), gazeboModelPos(2), gazeboModelPos(3), gazeboModelDist);
                    else
                        logPrint(4, 'identifyObjectsHere-'+string(zoneInspect), 4, '*red', "Failed to match object to gazebo model. Nearest Model Name: %s, ID'd Position: [%.2f, %.2f, %.2f], Nearest Model Position: [%.2f, %.2f, %.2f], Distance: %.2f", gazeboModelName, topCenterPosition(1), topCenterPosition(2), topCenterPosition(3), gazeboModelPos(1), gazeboModelPos(2), gazeboModelPos(3), gazeboModelDist);
                    end

                end

                objectData.ptCloud = objectPtCloud;
                objectData.label = label;
                objectData.bbox = bboxes(i, :);
                objectData.frustum = frustums(:, :, i);
                objectData.model = model;
                objectData.topCenterPosition = topCenterPosition;
                objectData.pickPose = pickPose;

                if this.optns{'comparePicksWithGazebo'}
                    objectData.gazeboMatch = struct();
                    objectData.gazeboMatch.modelName = gazeboModelName;
                    objectData.gazeboMatch.modelPos = gazeboModelPos;
                    objectData.gazeboMatch.dist = gazeboModelDist;
                end

                objects{i} = objectData;

            end


            objects = objects(~cellfun('isempty', objects));


            this.debugDrawFullContext(objects, W_T_ptCloud, W_T_R, R_T_C, frustums, zoneInspect + ": Full Detection Context");

            logPrint(2, 'identifyObjectsHere-'+string(zoneInspect), 3, '', "%d objects will be picked. %d skipped.", size(objects, 1), numOfObjects - size(objects, 1));
        end

        % Retrieves a point cloud from ROS and transforms it to the base link coordinate frame
        %
        % Receives a point cloud message, extracts its XYZ and RGB data, and transforms
        % the point cloud to the robot's base link coordinate system using ROS transform tree
        %
        % Returns:
        %   ptCloud_tform - Point cloud transformed to base_link coordinate frame
        %   ptCloud       - Original point cloud with XYZ and RGB data
        %   R_T_C         - Transform from robot base link to camera link
        %   C_T_R         - Inverse transform from camera link to robot base link
        function [ptCloud_tform, ptCloud, R_T_C, C_T_R] = getPointCloud(this)
            % Get point cloud
            pc = receive(this.robotHandle.pt_cloud_sub);

            % Get transform from base_link to camera_depth_link from ROS TF tree at point cloud timestamp.
            % ROS buffers position data for a few seconds, allowing retrieval of previous positions.
            % This eliminates the need to remain stationary while waiting for transforms.
            % Without buffering, rapid movement would result in incorrect transforms and misaligned
            % point clouds unless artificial delays were added to the code.
            R_T_C_tfmsg = getTransform(this.robotHandle.tftree, 'base_link', pc.Header.FrameId, pc.Header.Stamp, 'Timeout', this.robotHandle.tf_listening_time);
            R_T_C = rosReadTransform(R_T_C_tfmsg, OutputOption = "single"); % Converts rosmsg to MATLAB transform
            C_T_R = inv(R_T_C); % No need to ask ros for the inverse transform, just invert the matrix.

            % Extract xyz points
            xyz = rosReadXYZ(pc, "PreserveStructureOnRead", true);
            rgb = rosReadRGB(pc, "PreserveStructureOnRead", true);
            % Points -> MATLAB pointCloud object
            ptCloud = pointCloud(xyz, "Color", rgb);

            % Transform point cloud to base_link
            tform = rigidtform3d(R_T_C);

            ptCloud_tform = pctransform(ptCloud, tform);
        end


        % Merges multiple point clouds from different gripper positions to create a comprehensive scene representation
        %
        % Acquires an initial point cloud, moves the gripper to multiple predefined locations,
        % captures additional point clouds, and merges them. The resulting point cloud is cropped
        % and processed to separate planar and non-planar points.
        %
        % Inputs:
        %   locations - Cell array of gripper movement locations (e.g., {'f', 'l', 'r'} or "all")
        %
        % Outputs:
        %   ptCloud_pic    - Cropped point cloud representing the area of interest
        %   nonPlane_pic   - Subset of point cloud excluding the dominant plane (e.g., table surface)
        %   ptCloud_world  - Merged point cloud in the world coordinate frame
        function [ptCloud_pic, nonPlane_pic, ptCloud_world] = getSeveralPointClouds(this, locations)
            % Extract point cloud and transforms in both directions
            [ptCloud_world, ~, ~, ~] = this.getPointCloud();
            logPrint(2, 'getMergedPTC', 2, '', "Initial point cloud acquired.");
            % Gather the x and y limits of this very first point cloud and store them
            % TO_ENHANCE: convert to a function
            xlim_min = ptCloud_world.XLimits(1, 1);
            xlim_max = ptCloud_world.XLimits(1, 2);

            ylim_min = ptCloud_world.YLimits(1, 1);
            ylim_max = ptCloud_world.YLimits(1, 2);

            % Z-limits are hard-coded and subject to modification
            zlim_min = -0.13; zlim_max = 0.4;

            % From these limits create a region of interest (ROI)
            roi = [xlim_min xlim_max ylim_min ylim_max zlim_min zlim_max];

            % Crop point cloud wrt to z values
            indices = findPointsInROI(ptCloud_world, roi);
            ptCloud_world = select(ptCloud_world, indices);
            % Update xyz limits given the new point cloud
            xlim_min = ptCloud_world.XLimits(1, 1);
            xlim_max = ptCloud_world.XLimits(1, 2);

            ylim_min = ptCloud_world.YLimits(1, 1);
            ylim_max = ptCloud_world.YLimits(1, 2);

            % Hard-coded z-limits
            zlim_min = -0.13; zlim_max = 0.4;

            % New ROI for object
            pic_roi = [xlim_min xlim_max ylim_min ylim_max zlim_min zlim_max];

            % Hard-coded ROI for table
            table_roi = [-0.3 2 -1.5 1.5 zlim_min zlim_max];

            % Gripper pose
            mat_R_T_G = get_gripper_pose(this.optns);
            cur_gripper_location = mat_R_T_G;

            % Compute locations for the arm to move to: (f-forward, l-left, r-right, b-back)
            if size(locations, 1) == 0
                locations = {'f', 'l', 'r'};
            elseif locations == "all"
                % We repeat locations. First four have no angle offset, last four will
                locations = {'f', 'b', 'l', 'r', 'f', 'b', 'l', 'r', };
            end

            currentFuture = parfeval(@(a) a, 1, ptCloud_world); % Create a future object that returns immediately

            % Move the arm to each of these locations and take pc pic
            for iter = 1:length(locations)
                logPrint(2, 'getMergedPTC', 3, '', "Point Cloud Location %d", iter);
                % a) Move arm to ith location:

                logPrint(3, 'getMergedPTC', 4, '', "Moving to %s location...", locations{iter});
                % These gripper motions are planar (no gripper rotation)
                if iter < 5
                    displaceG = displace_gripper(mat_R_T_G, this.optns, locations{iter}, 0.12);

                    % These motions rotate the gripper inwards at end of displacement
                    % to center object.
                elseif iter > 5
                    displaceGA = displace_gripper(mat_R_T_G, this.optns, locations{iter}, 0.12, 1, 0.05);
                end

                % b) Get point cloud (above) at that location
                [ptCloud_recent, ~, ~, ~] = this.getPointCloud();
                logPrint(3, 'getMergedPTC', 4, '', "Point Cloud acquired.");
                logPrint(4, 'getMergedPTC', 4, '', "Waiting for previous point cloud merging to finish...");
                wait(currentFuture);
                ptCloud_world = fetchOutputs(currentFuture);
                logPrint(4, 'getMergedPTC', 4, '', "Complete. Dispatching (async) this point cloud to merge with previous one.");
                currentFuture = parfeval(@threadedMergePtClouds, 1, ptCloud_world, ptCloud_recent, table_roi);
            end

            % To resent move arm back to original position (per zone)
            logPrint(2, 'getMergedPTC', 2, '', "Return to start position.");
            moveTo(cur_gripper_location, this.optns);

            % Cropping the merged point cloud to the area of our picture

            % Compute indeces of points inside new ROI
            indices = findPointsInROI(ptCloud_world, pic_roi);

            % Keep point clouds with those indeces (plane + objects)
            ptCloud_pic = select(ptCloud_world, indices);

            % Create nonPlane and plane objects that index points that below to those entities

            % Horizontal Plane parameters
            planeThickness = .001;
            normalVector = [0, 0, 1];
            maxPlaneTilt = 5;

            % TODO: Fit the horizontal plane given ptCloud_pic and the three arguments above.
            [param, planeIdx, nonPlaneIdx] = pcfitplane(ptCloud_pic, planeThickness, normalVector, maxPlaneTilt);

            % Create indexed entities
            plane_pic = select(ptCloud_pic, planeIdx);
            nonPlane_pic = select(ptCloud_pic, nonPlaneIdx);
            logPrint(4, 'getMergedPTC', 2, '', "Removed table top from point cloud.");
        end

        % Asynchronously retrieve and process a labeled image from the robot's RGB camera
        %
        % Receives an RGB image from the robot's camera and initiates an asynchronous YOLO detection
        %
        % Returns:
        %   future      - Parallel future for the YOLO detection result
        %   imgMsg      - ROS image message received from the camera
        %   myImg       - Processed image read from the ROS message
        %
        % See also: threadedYOLODetection, rosReadImage
        function [future, imgMsg, myImg] = getLabeledImageAsync(this)
            imgMsg = receive(this.robotHandle.rgb_sub);
            myImg = rosReadImage(imgMsg, "PreserveStructureOnRead", true);

            future = parfeval(@threadedYOLODetection, 5, myImg, this.imageDetectors);
        end

        % Filter point cloud to include only points within a 3D frustum volume
        %
        % Inputs:
        %   ptCloud - Point cloud to filter
        %   frustum - 8x3 matrix of frustum corner points defining the volume
        %
        % Outputs:
        %   subsetCloud - Point cloud containing only points inside the frustum
        %
        % Filters a point cloud by testing each point against the planes
        % defining the frustum's bounding volume
        function subsetCloud = filterPointCloudByBBox(~, ptCloud, frustum)
            % Define faces of frustum (each row indices into corners)
            faces = [1 2 3; 1 3 4; 5 6 7; 5 7 8; 1 2 6; 2 3 7; 3 4 8; 4 1 5];
            corners = frustum;

            % Compute centroid to orient normals inward
            centroid = mean(corners, 1);

            % Preallocate plane coefficients (8 planes, 4 coeffs each)
            numFaces = size(faces, 1);
            planes = zeros(numFaces, 4);

            % Build each plane: [n_x n_y n_z d]
            for f = 1:numFaces
                idx = faces(f, :);
                p1 = corners(idx(1), :);
                p2 = corners(idx(2), :);
                p3 = corners(idx(3), :);
                n = cross(p2 - p1, p3 - p1); % 1x3 row vector
                n = n / norm(n);
                d = -dot(n, p1);
                % flip to point inward
                if dot(n, centroid) + d < 0
                    n = -n; d = -d;
                end

                planes(f, :) = [n d]; % use row vector n
            end

            % Flatten point cloud locations
            locs = ptCloud.Location;
            flat = reshape(locs, [], 3);

            % Test all points against all half-spaces
            maskAll = true(size(flat, 1), 1);

            for f = 1:numFaces
                n = planes(f, 1:3);
                d = planes(f, 4);
                maskAll = maskAll & (flat * n' + d >= 0);
            end

            % Select and return subset via built-in select
            indices = find(maskAll);
            subsetCloud = select(ptCloud, indices);
        end

        % Convert bounding box to 3D frustum corners in world coordinates
        %
        % Transforms 2D image bounding boxes into 3D frustum corners, accounting for camera
        % intrinsics, camera pose, and specified near and far plane distances.
        %
        % Inputs:
        %   camInfo  - ROS sensor_msgs/CameraInfo struct with camera calibration parameters
        %   camPose  - rigid3d transformation representing camera-to-world coordinate frame
        %   bboxes   - Nx4 array of bounding boxes [x, y, width, height] in pixel coordinates
        %   zNear    - Near plane distance in meters
        %   zFar     - Far plane distance in meters
        %
        % Outputs:
        %   frustumCornersWorld - 8x3xN array of world-frame frustum corner points
        %                         Ordered as [nearTL; nearTR; nearBR; nearBL; farTL; farTR; farBR; farBL]
        function frustumCornersWorld = bboxToFrustum(~, camInfo, camPose, bboxes, zNear, zFar)
            frustumCornersWorld = zeros(8, 3, size(bboxes, 1));

            for i = 1:size(bboxes, 1)
                bbox = bboxes(i, :);
                % Build intrinsic matrix
                K = reshape(camInfo.K, 3, 3)';
                fx = K(1, 1); fy = K(2, 2);
                cx = K(1, 3); cy = K(2, 3);

                % Pixel coordinates of bbox corners
                u = [bbox(1), bbox(1) + bbox(3), bbox(1) + bbox(3), bbox(1)];
                v = [bbox(2), bbox(2), bbox(2) + bbox(4), bbox(2) + bbox(4)];

                % Convert pixels to normalized camera directions
                dirs = [(u(:) - cx) / fx, (v(:) - cy) / fy, ones(4, 1)];
                % Normalize directions (optional, for consistent scaling)
                dirs = dirs ./ vecnorm(dirs, 2, 2);

                % Compute 3D points in camera frame at near and far planes
                ptsNearCam = dirs * zNear;
                ptsFarCam = dirs * zFar;

                % Combine into 8 corners
                ptsCam = [ptsNearCam; ptsFarCam]; % 8x3

                % Transform to world frame using camPose
                frustumCornersWorld(:, :, i) = transformPointsForward(camPose, ptsCam);
            end

        end

        % Determine the optimal pick pose for cylindrical or cuboid objects
        %
        % Attempts to fit a cylinder or cuboid model to a point cloud and generate
        % an appropriate pick pose based on the object's orientation and type.
        % For cylindrical objects (cans/bottles), determines if they are upright or lying down.
        % For pouches, fits a cuboid and aligns with its principal axes.
        %
        % Inputs:
        %   objectPtCloud - Point cloud representing the object to be picked
        %   label         - String label identifying the object type ('can', 'bottle', 'pouch')
        %   bbox          - Bounding box of the object in image coordinates [x, y, width, height]
        %
        % Outputs:
        %   model             - Fitted geometric model (cylinder or cuboid) containing parameters
        %   topCenterPosition - 3D coordinates [x,y,z] of the object's top center point
        %   pickPose          - 4x4 homogeneous transformation matrix for the optimal pick pose
        %   newLabel          - Updated label with prefix 'v' (vertical) or 'h' (horizontal) for cylinders
        function [model, topCenterPosition, pickPose, newLabel] = fitPickPoseToObject(this, objectPtCloud, label, bbox)
            try

                if strcmp(label, "can") || strcmp(label, "bottle")
                    % Handle cylindrical objects (cans and bottles)
                    % First determine if object is upright or lying down based on visual features

                    logPrint(3, 'fitPickPose-'+string(label), 5, '', "Attemping to fit a vertical cylinder to point cloud.");

                    orientation = "unknown";

                    % Check aspect ratio of bounding box to guess initial orientation
                    % Wide/short boxes (ratio > 1.5) or tall/narrow boxes (ratio < 0.75) suggest horizontal orientation
                    if (bbox(3) / bbox(4) > 1.5) || (bbox(3) / bbox(4) < 0.75)
                        orientation = "horizontal";
                    end

                    % Try fitting vertical cylinder first unless we're confident it's horizontal
                    if strcmp(orientation, "unknown")
                        % Fit cylinder with axis constrained to vertical (Z) direction
                        [model, inIndices, outIndices, meanError] = pcfitcylinder(objectPtCloud, 0.005, [0 0 1]);
                        logPrint(4, 'fitPickPose-'+string(label), 6, '', "Radius: %.3f, Height: %.3f, Mean Error: %.3f, Num Inside Pts: %d, Num Outside Pts: %d", model.Radius, model.Height, meanError, size(inIndices, 1), size(outIndices, 1));
                    end

                    % If vertical fit fails or looks poor, try unconstrained cylinder fit
                    if strcmp(orientation, "horizontal") || model.Radius < 0.01 || model.Radius > 0.1 || meanError > 0.003
                        logPrint(4, 'fitPickPose-'+string(label), 6, '', "Vertical Cylinder fit was not good enough. Trying again without the vertical constraint.");

                        [model, inIndices, outIndices, meanError] = pcfitcylinder(objectPtCloud, 0.005);
                        logPrint(4, 'fitPickPose-'+string(label), 6, '', "New Cylinder: Radius: %.3f, Height: %.3f, Mean Error: %.3f, Num Inside Pts: %d, Num Outside Pts: %d", model.Radius, model.Height, meanError, size(inIndices, 1), size(outIndices, 1));
                    end

                    % Validate cylinder dimensions are reasonable (1-10cm radius)
                    if model.Radius < 0.01 || model.Radius > 0.1
                        logPrint(3, 'fitPickPose-'+string(label), 6, 'red', "Object does not match cylinder fit. No pick pose will be created.");
                        model = [];
                        topCenterPosition = [];
                        pickPose = [];
                        newLabel = label;
                        return;
                    end

                    % Determine final orientation by checking height difference between cylinder ends
                    % If ends differ by >4cm in height, consider it vertical
                    if (abs(model.Parameters(6) - model.Parameters(3)) > 0.04) && orientation == "unknown"
                        orientation = "vertical";
                    else
                        orientation = "horizontal";
                    end

                    logPrint(4, 'fitPickPose-'+string(label), 5, '', "This object is likely %s.", orientation);

                    % Generate pick pose aligned with cylinder axis and positioned at top center
                    logPrint(3, 'fitPickPose-'+string(label), 5, '', "Generating a pick pose at the top center of the object. This even works for lying cylinders.");
                    logPrint(3, 'fitPickPose-'+string(label), 6, '', "Center Position: [%.3f, %.3f, %.3f]", -model.Center(2), model.Center(1), model.Center(3));

                    % Initial pick pose with gripper pointing down (-Z)
                    pickPose = transl(-model.Center(2), model.Center(1), objectPtCloud.ZLimits(1, 2)) * trotx(-pi);

                    logPrint(3, 'fitPickPose-'+string(label), 6, '', "Pick Position: [%.3f, %.3f, %.3f]", -model.Center(2), model.Center(1), objectPtCloud.ZLimits(1, 2));

                    % Store top center position for visualization/debugging
                    topCenterPosition = [model.Center(1), model.Center(2), objectPtCloud.ZLimits(1, 2)];

                    % Transform pick pose from world frame to robot base frame
                    W_T_R2 = ros2MLRobotPose(get_model_pose('robot', this.optns), 1, 1, this.optns);
                    pickPose = W_T_R2 \ pickPose;
                    % Offset gripper position slightly above object
                    pickPose(3, 4) = pickPose(3, 4) - 0.07;

                    label = char(label);

                    % Update label and adjust gripper orientation based on cylinder orientation
                    if strcmp(orientation, "horizontal")
                        % Add 'h' prefix for horizontal objects
                        label(1) = upper(label(1));
                        label = "h" + label;
                        % Calculate gripper rotation to align with cylinder axis
                        v = model.Parameters(4:6) - model.Parameters(1:3);
                        vx = v(1);
                        vy = v(2);
                        v(1) = -vy;
                        v(2) = vx;
                        horizontalAngle = atan(v(1) / v(2));
                        logPrint(4, 'fitPickPose-'+string(label), 5, '', "Horizontal Angle: %.2f radians from the +Y axis", horizontalAngle);
                        pickPose = pickPose * trotz(horizontalAngle);
                    elseif strcmp(orientation, "vertical")
                        % Add 'v' prefix for vertical objects
                        label(1) = upper(label(1));
                        label = "v" + label;
                    end

                    newLabel = label;
                    logPrint(4, 'fitPickPose-'+string(label), 5, '', "Object relabeled to %s.", label);

                elseif strcmp(label, "pouch")
                    % Handle pouch objects by fitting a cuboid model
                    logPrint(3, 'fitPickPose-'+string(label), 5, '', "Attemping to fit a cuboid to point cloud.");
                    model = pcfitcuboid(objectPtCloud);

                    % Log cuboid parameters for debugging
                    logPrint(4, 'fitPickPose-'+string(label), 6, '', "Center: [%.3f, %.3f, %.3f], Dimensions: [%.3f, %.3f, %.3f], Orientation: [%.3f, %.3f, %.3f]", -model.Center(2), model.Center(1), model.Center(3), model.Dimensions(2), model.Dimensions(1), model.Dimensions(3), model.Orientation(2), model.Orientation(1), model.Orientation(3));

                    logPrint(3, 'fitPickPose-'+string(label), 5, '', "Generating a pick pose at the top center of the object.");

                    % Calculate top center position using point cloud Z limits
                    topCenterPosition = [model.Center(1), model.Center(2), objectPtCloud.ZLimits(1, 2)];

                    % Generate initial pick pose with gripper pointing down
                    pickPose = transl(-topCenterPosition(2), topCenterPosition(1), topCenterPosition(3)) * trotx(-pi);

                    % Transform to robot base frame
                    W_T_R2 = ros2MLRobotPose(get_model_pose('robot', this.optns), 1, 1, this.optns);
                    pickPose = W_T_R2 \ pickPose;
                    pickPose(3, 4) = pickPose(3, 4) - 0.07;

                    % Align gripper with cuboid orientation
                    horizontalAngle = deg2rad(model.Orientation(3));
                    logPrint(4, 'fitPickPose-'+string(label), 5, '', "Horizontal Angle: %.2f radians from the +Y axis", horizontalAngle);
                    pickPose = pickPose * trotz(horizontalAngle);

                    newLabel = label;

                    logPrint(3, 'fitPickPose-'+string(label), 6, '', "Pick Position: [%.3f, %.3f, %.3f]", -model.Center(2), model.Center(1), objectPtCloud.ZLimits(1, 2));
                end

            catch
                % Return empty results if fitting fails
                model = [];
                topCenterPosition = [];
                pickPose = [];
                newLabel = label;
                return;
            end

        end

        % Find the nearest Gazebo model to a given position with efficient caching mechanism
        %
        % Efficiently retrieves the closest Gazebo model to a specified position by
        % maintaining a cache of model names and positions. This significantly reduces
        % network requests and improves performance by avoiding repeated Gazebo queries.
        %
        % Inputs:
        %   pos - 1x3 vector [X,Y,Z] specifying the query position in world coordinates
        %
        % Outputs:
        %   modelName - String name of the closest Gazebo model found
        %   modelPos  - 3D position vector [X,Y,Z] of the closest model in world coordinates
        %   dist      - Euclidean distance (in meters) from the query position to the model
        %
        % Cache Details:
        %   - Stores model information for 10 seconds before refreshing
        %   - Uses persistent variables to maintain cache between function calls
        %   - Significantly reduces network overhead and computation time
        function [modelName, modelPos, dist] = debugGetNearestModelFromGazebo(this, pos)
            % Persistent variables for caching mechanism
            persistent cachedModelNames;     % Stores all Gazebo model names
            persistent cachedModelPoses;     % Stores corresponding model positions
            persistent cachedModelPosesLastUpdated;  % Timestamp of last cache update

            nowtime = now();

            % Check if cache needs refreshing (empty or older than 10 seconds)
            % This caching mechanism significantly reduces network requests to Gazebo
            % and improves performance when multiple queries are made in succession
            if isempty(cachedModelPoses) || isempty(cachedModelPosesLastUpdated) || isempty(cachedModelNames) || (nowtime - cachedModelPosesLastUpdated > 10000)
                cachedModelPosesLastUpdated = nowtime;

                % Retrieve fresh list of all model names from Gazebo
                allModels = getModels(this.optns);
                cachedModelNames = allModels.ModelNames;  % Store as cell array of char arrays
                cachedModelPoses = cell(size(cachedModelNames, 1), 1);

                % Populate cache with current model positions
                for i = 1:size(cachedModelNames, 1)
                    % Retrieve individual model pose from Gazebo
                    model = get_model_pose(cachedModelNames{i}, this.optns);
                    % Extract position components from pose structure
                    pStruct = model.Pose.Position;  % Contains X, Y, Z fields
                    % Store position as 3D vector
                    cachedModelPoses{i} = [pStruct.X, pStruct.Y, pStruct.Z];
                end
            end

            % Initialize output variables with default values
            modelName = '';  % Empty string for model name
            modelPos  = [NaN, NaN, NaN];  % NaN vector for position
            dist      = inf;  % Infinite distance as initial comparison value

            % Search through cached models to find the nearest one
            % Starting from index 12 to skip certain system models
            for i = 12:size(cachedModelNames, 1)
                % Get current model's position from cache
                currentPos = cachedModelPoses{i};
                % Calculate Euclidean distance to query position
                d = sqrt(sum((currentPos - pos) .^ 2));
                % Update outputs if this model is closer than previous best
                if d < dist
                    dist      = d;
                    modelName = cachedModelNames{i};
                    modelPos  = currentPos;
                end
            end
        end

        % Visualize 3D camera frustums by drawing their edges in 3D space
        %
        % Draws the edges of near and far planes for each camera frustum in the input array.
        % A frustum represents the visible volume of space that a camera can see,
        % shaped like a truncated pyramid. Each frustum is defined by 8 corner points
        % that form the near (close to camera) and far (away from camera) rectangular planes.
        %
        % Frustums are represented as 8x3xN arrays with points ordered as follows:
        % - Points 1-4: Near plane corners (Top-Left, Top-Right, Bottom-Right, Bottom-Left)
        % - Points 5-8: Far plane corners (Top-Left, Top-Right, Bottom-Right, Bottom-Left)
        %
        % Inputs:
        %   frustums - 8x3xN array of frustum corner points, where:
        %              - 8 represents the number of corners per frustum
        %              - 3 represents XYZ coordinates for each corner
        %              - N represents the number of frustums to draw
        function debugDrawFrustums(~, frustums)
            % Corner point ordering:
            % Near plane: [1=TopLeft; 2=TopRight; 3=BottomRight; 4=BottomLeft]
            % Far plane:  [5=TopLeft; 6=TopRight; 7=BottomRight; 8=BottomLeft]

            for i = 1:size(frustums,3)
                % Extract corner points for near and far planes of current frustum
                nearPts = frustums(1:4, :, i);  % First 4 points form near plane
                farPts  = frustums(5:8, :, i);  % Last 4 points form far plane

                % Draw all 12 edges of the frustum (4 near edges, 4 far edges, 4 connecting edges)
                for j = 1:4
                    % Get current and next points for both near and far planes
                    ni = nearPts(j, :);         % Current near point
                    fi = farPts(j, :);          % Current far point
                    nj = nearPts(mod(j,4)+1, :); % Next near point (wraps around to 1)
                    fj = farPts(mod(j,4)+1, :);  % Next far point (wraps around to 1)

                    % Draw edge on near plane (red line)
                    plot3([ni(1), nj(1)], [ni(2), nj(2)], [ni(3), nj(3)], 'r-');
                    % Draw edge on far plane (red line)
                    plot3([fi(1), fj(1)], [fi(2), fj(2)], [fi(3), fj(3)], 'r-');
                    % Draw connecting edge between near and far planes (red line)
                    plot3([ni(1), fi(1)], [ni(2), fi(2)], [ni(3), fi(3)], 'r-');
                end
                hold on;  % Maintain current plot while adding more lines
            end
        end

        % Visualize full debugging context for detected objects and camera positioning
        %
        % Generates a comprehensive 3D visualization plot showing:
        % - Raw point cloud data from camera
        % - Detected objects with their fitted geometric models
        % - Object labels and top center positions
        % - Robot base position in world coordinates
        % - Camera position relative to robot base
        % - Camera view frustums showing field of view
        %
        % This visualization helps debug spatial relationships between
        % all components in the scene and verify correct object detection.
        %
        % Inputs:
        %   objects            - Cell array of detected objects containing fitted geometric
        %                       models and properties like position and labels
        %   ptCloud            - Point cloud data captured from camera
        %   robotBasePose      - Pose (position + orientation) of the robot base in world frame
        %   cameraPoseFromBase - Transform describing camera pose relative to robot base frame
        %   frustums           - Array of camera view frustums defining visible volumes
        %   figureTitle        - Optional custom title for the debug figure window
        %
        % Note: Point cloud and robot base pose must be specified in the same coordinate frame
        %       for proper spatial alignment in the visualization
        function debugDrawFullContext(this, objects, ptCloud, robotBasePose, cameraPoseFromBase, frustums, figureTitle)
            % Set default figure title if none provided
            if isempty(figureTitle)
                figureTitle = 'Debug Display: Whole Zone';
            end

            % Create new figure and display point cloud
            figure;
            pcshow(ptCloud);
            hold on;

            % Iterate through detected objects and visualize each one
            for i = 1:size(objects,1)
                obj = objects{i};
                % Plot the geometric model (e.g. cylinder) fitted to the object
                plot(obj.model);
                hold on;
                % Plot marker at top center of object and add text label
                scatter3(obj.topCenterPosition(1), obj.topCenterPosition(2), obj.topCenterPosition(3), 150, [1 0 1], 'filled');
                text(obj.topCenterPosition(1), obj.topCenterPosition(2), obj.topCenterPosition(3), obj.label, "Color", [1 1 1], "FontWeight", "bold", "HorizontalAlignment", "right", "VerticalAlignment", "bottom");
                hold on;
            end

            % Extract position vectors from poses
            % wtr = world-to-robot base position
            wtr = [robotBasePose.Pose.Position.X;robotBasePose.Pose.Position.Y;robotBasePose.Pose.Position.Z];
            % rtc = robot base-to-camera position
            rtc = [cameraPoseFromBase.Transform.Translation.X;cameraPoseFromBase.Transform.Translation.Y;cameraPoseFromBase.Transform.Translation.Z];
            % wtc = world-to-camera position (by vector addition)
            wtc = wtr + rtc;

            % Prepare coordinate arrays for scatter plot of robot base and camera positions
            u = [wtr(1) wtc(1)];  % X coordinates
            v = [wtr(2) wtc(2)];  % Y coordinates
            Z = [wtr(3) wtc(3)];  % Z coordinates

            % Add axis labels for spatial reference
            axis on;
            xlabel("X"); ylabel("Y"); zlabel("Z");

            % Plot and label robot base and camera positions
            hold on;
            scatter3(u, v, Z, 100, [1 0 0; 1 0 0], 'filled');
            text(u, v, Z, ["Robot Base", "Camera"], "Color", "red", "FontWeight", "bold", "HorizontalAlignment", "center", "VerticalAlignment", "bottom");

            % Add visualization of camera view frustums
            hold on;
            this.debugDrawFrustums(frustums);

            % Set figure title and release plot hold
            title(figureTitle); hold off;
        end
    end

end
