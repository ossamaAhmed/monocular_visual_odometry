function M = getProjectionMat(R, t, cameraParams)
    % R [3 x 3]
    % t [3 x 1]
    % cameraParams, cameraParameters object with IntrinsicMatrix
    K = cameraParams.IntrinsicMatrix';
    M = K*[R, t];
end

