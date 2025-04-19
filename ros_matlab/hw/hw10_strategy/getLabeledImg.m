function [bboxes, scores, labels, numObjects, annotatedImage] = getLabeledImg(myImg, general_detector)
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
    %% According to strategy leverage different detectors...
    trainedYoloNet = general_detector.detector;

    %% TODO: Detect objects using yolo. Output bboxes, scores, labels. Threshold of 0.7
    [bboxes,scores,labels] = detect(trainedYoloNet,myImg,Threshold=0.7);

    %% TODO: Visualize the detected objects' bounding boxes by calling insertObjectAnnotation and save to annotatedImage

    annotatedImage = insertObjectAnnotation(im2uint8(myImg), ...
                                            'Rectangle',...
                                            bboxes,...
                                            string(labels)+":"+string(scores),...
                                            'Color','cyan');

    %% Specify percentage of acceptable bounding box
    numObjects = size(bboxes,1);
end



