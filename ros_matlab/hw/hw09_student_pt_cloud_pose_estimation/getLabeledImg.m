function [bboxes, scores, labels, numObjects, myImg, annotatedImage] = getLabeledImg(zone, optns)
% ------------------------------------------------------------------------
% Takes a picture by accessing a subscriber, loads the dector and labels
% the image taken, and displays it.
% 
% Input: 
%   optns - to access the subscriber to take a pic through the
%                rosClass
% Outputs:
%   bboxes - P-by-4 matrix defining P bounding boxes. Each row of bboxes
%              contains a four-element vector, [x, y, width, height]. This
%              vector specifies the upper-left corner and size of a bounding
%              box in pixels 
%   scores - confidence scores for each bounding box (PX1)
%   labels - labels assigned to the bounding boxes (PX1)
%   numObjects - number of objects detected ( would also be P )
%   myImg - image taken without any labels
%   annotatedImage - myImg but with bounding boxes, scores, and labels for
%               each objects detected
% ------------------------------------------------------------------------
    %% Get the ROS Class handel
    r = optns{'rHandle'};

    %% Take picture and Read the Image
    pause(10);
    disp("Taking picture...")

    rosImg  = receive(r.rgb_sub);
    myImg   = rosReadImage(rosImg,"PreserveStructureOnRead",true);
    
    %% Bounding Boxes
    disp("Computing bounding boxes, scores, and labels...")

    %% According to strategy leverage different detectors...
    pretrained = r.general_detector;
    trainedYoloNet = pretrained.detector;

    %% TODO: Detect objects using yolo. Output bboxes, scores, labels. Threshold of 0.7
    [bboxes,scores,labels] =    

    %% TODO: Visualize the detected objects' bounding boxes by calling insertObjectAnnotation and save to annotatedImage
    disp("Drawing bounding boxes...")
    annotatedImage = 

    % Display
    if optns{'debug'}
        figure(1), imshow(annotatedImage);
    end

    %% Specify percentage of acceptable bounding box
    numObjects = size(bboxes,1);

end



