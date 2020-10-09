function [filtered_points3d, valid_idx] = filterTriangulatedLandmarks(points3d)
% Takes points 3d [N x 3]
    global landmark_upperbound
    valid_idx = (points3d(:,3) > 0) & (points3d(:,3) < landmark_upperbound);
    filtered_points3d = points3d(valid_idx,:);
end

