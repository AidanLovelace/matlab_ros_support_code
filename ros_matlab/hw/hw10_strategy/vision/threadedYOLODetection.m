function [bboxes, labels, scores, numObjects, annotatedImage] = threadedYOLODetection(myImg, detectorsInPool)
    detectors = detectorsInPool.Value();

    bboxes = [];
    scores = [];
    labels = [];

    parfor i = 1:length(detectors)
        this_detector = detectors{i};
        [mybboxes, myscores, mylabels] = detect(this_detector, myImg, Threshold = 0.7);
        bboxes = [bboxes; mybboxes];
        scores = [scores; myscores];
        labels = [labels; mylabels];
    end

    image = myImg;
    numObjects = size(bboxes, 1);
    annotatedImage = insertObjectAnnotation(im2uint8(myImg), ...
        'Rectangle', ...
        bboxes, ...
        string(labels) + ": "+string(scores), ...
        'Color', 'cyan');
end