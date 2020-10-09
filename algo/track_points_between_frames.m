function [matched_keypoints, valid_mask] = ...
    track_points_between_frames(prev_keypoints, prev_fram, curr_frame)
    %fprintf("points to track: %d \n", size(prev_keypoints, 1));
    tracker = vision.PointTracker('MaxBidirectionalError',6, ...
                                  'BlockSize', [21,21], ...
                                  'NumPyramidLevels', 3);
    initialize(tracker, prev_keypoints, prev_fram);
    [keypts, valid_mask, ~] = tracker(curr_frame);
    matched_keypoints = keypts(valid_mask, :);
    %fprintf("points tracked: %d \n", sum(valid_mask));
end

