function [keypoints] = customHarris(frame)
% Randomly chosen parameters that seem to work well - can you find better
% ones?
global harris_patch_size;
global harris_kappa;
global num_keypoints;
global nonmaximum_supression_radius;



%% Part 1 - Calculate Harris scores

harris_scores = harris(frame, harris_patch_size, harris_kappa);
assert(min(size(harris_scores) == size(frame)));

%% Part 2 - Select keypoints

keypoints = selectKeypoints(...
    harris_scores, num_keypoints, nonmaximum_supression_radius);

keypoints = flipud(keypoints)';
end