function resulting_keypoints = removeClose2DPoints(...
    first_keypoints, second_keypoints, slack)
    %removes close points from the first set to the second set and returns
    %the rest of the first set.
    resulting_keypoints = [];
    for k = 1:size(first_keypoints,1)
        x_range = [first_keypoints(k,1)-slack, ...
            first_keypoints(k,1)+slack];
        y_range = [first_keypoints(k,2)-slack, ...
            first_keypoints(k,2)+slack];
        mask = second_keypoints(:, 1)>x_range(1) &...
               second_keypoints(:, 1)<x_range(2) &...
               second_keypoints(:, 2)>y_range(1) &...
               second_keypoints(:, 2)<y_range(2);
        
        if sum(mask) == 0
            resulting_keypoints = [resulting_keypoints; first_keypoints(k, :)];
        end
    end
end