function [matchedPoints1, matchedPoints2] = ...
    getInitialCorrespondances(img0, img1, img2)
    
    global matlabfuncs;
    global min_quality;
    global gauss_filter_size;
    
    if matlabfuncs

        points1 = detectHarrisFeatures(img0, ...
            'MinQuality', min_quality, ...
            'FilterSize', gauss_filter_size);

        [~,imagePoints1] = extractFeatures(img0,points1);
        imagePoints1 = imagePoints1.Location;
    else
        imagePoints1 = customHarris(img0);
    end

    % Set up KLT
    tracker = vision.PointTracker('MaxBidirectionalError', 1, ...
                                  'NumPyramidLevels', 5);
                              
    initialize(tracker, imagePoints1, img0); step(tracker, img1);
    [imagePoints2, validIdx] = step(tracker, img2);
    
    imagePoints1 = imagePoints1(validIdx, :);
    imagePoints2 = imagePoints2(validIdx, :);
  
    matchedPoints1 = cornerPoints(imagePoints1);
    matchedPoints2 = cornerPoints(imagePoints2);
end

