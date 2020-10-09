function [keypoints, p_W_landmarks, matchedPoints1, matchedPoints2, T_WC] = ...
    runSFM(matchedPoints1,matchedPoints2)

    global K;
    global matlabfuncs;

    cameraParams = cameraParameters('IntrinsicMatrix',K');
    [E, inliers] = estimateEssentialMatrix(matchedPoints1, ...
                                           matchedPoints2, ...
                                           cameraParams, ...
                                           'Confidence', 99.99);
    % Get inlier points
    matchedPoints1 = matchedPoints1(inliers, :);
    matchedPoints2 = matchedPoints2(inliers, :);

    if matlabfuncs
        [orient, loc] = relativeCameraPose(E, cameraParams, matchedPoints1, matchedPoints2);
        [R, t] = cameraPoseToExtrinsics(orient, loc);
    else
        [orient, loc] = relativeCameraPose(E, cameraParams, matchedPoints1, matchedPoints2);
        [R, t] = cameraPoseToExtrinsics(orient, loc);
    end
    
    % Set up projection matrices M
    M1 = getProjectionMat(eye(3), [0 0 0]', cameraParams);
    M2 = getProjectionMat(R, t', cameraParams);

    matchedPoints1 = toHomogeneousPixelCoords(matchedPoints1.Location);
    matchedPoints2 = toHomogeneousPixelCoords(matchedPoints2.Location);

    points3D = linearTriangulation(matchedPoints1', ...
                                   matchedPoints2', ...
                                   M1, ...
                                   M2);

    points3D = toDehomogenousLandmarks(points3D)';

    [p_W_landmarks, valid_idx] = filterTriangulatedLandmarks(points3D);

    p_W_landmarks = single(p_W_landmarks);
    keypoints = single(matchedPoints1(valid_idx',1:2));
    T_WC = pose_refinement([R, t'], keypoints, p_W_landmarks, cameraParams);
    keypoints = single(matchedPoints2(valid_idx',1:2));
end

