function [T_WC, T_CW, best_inlier_mask, ...
    max_num_inliers_history] = ransacLocalization(...
    keypts_next, landmarks, K)


    use_p3p = true;
    tweaked_for_more = true;

    if use_p3p
        if tweaked_for_more
            num_iterations = 1000;
        else
            num_iterations = 200;
        end
        pixel_tolerance = 13;
        k = 3;
    else
        num_iterations = 2000;
        pixel_tolerance = 13;
        k = 6;
    end

    % Initialize RANSAC
    max_num_inliers_history = zeros(1, num_iterations);
    max_num_inliers = 0;
    best_guess_WC = [];
    best_guess = [];
    best_R = [];
    best_t = [];


    % RANSAC
    for i = 1:num_iterations
        % Model from k samples (DLT or P3P)
        [landmark_sample, idx] = datasample(...
            landmarks, k, 1, 'Replace', false);
        keypoint_sample = keypts_next(idx, :);

        if use_p3p
            % Backproject keypoints to unit bearing vectors.
            normalized_bearings = K\[keypoint_sample'; ones(1, 3)];
            for ii = 1:3
                normalized_bearings(:, ii) = normalized_bearings(:, ii) / ...
                    norm(normalized_bearings(:, ii), 2);
            end

            poses = p3p(landmark_sample', normalized_bearings);

            % Decode p3p output
            R_C_W_guess = zeros(3, 3, 4);
            t_C_W_guess = zeros(3, 1, 4);
            for ii = 0:3
                R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4)));
                t_W_C_ii = real(poses(:, (1+ii*4)));
                R_C_W_guess(:,:,ii+1) = R_W_C_ii';
                t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
            end
        else
            M_C_W_guess = estimatePoseDLT(...
                keypoint_sample', landmark_sample', K);
            R_C_W_guess = M_C_W_guess(:, 1:3);
            t_C_W_guess = M_C_W_guess(:, end);
        end

        % Count inliers:
        projected_points = projectPoints(...
            (R_C_W_guess(:,:,1) * landmarks') + ...
            repmat(t_C_W_guess(:,:,1), ...
            [1 size(landmarks, 1)]), K);
        difference = keypts_next' - projected_points;
        errors = sum(difference.^2, 1);
        is_inlier = errors < pixel_tolerance^2;

        % If we use p3p, also consider inliers for the alternative solution.
        if use_p3p
            projected_points = projectPoints(...
                (R_C_W_guess(:,:,2) * landmarks') + ...
                repmat(t_C_W_guess(:,:,2), ...
                [1 size(landmarks, 1)]), K);
            difference = keypts_next' - projected_points;
            errors = sum(difference.^2, 1);
            alternative_is_inlier = errors < pixel_tolerance^2;
            if nnz(alternative_is_inlier) > nnz(is_inlier)
                is_inlier = alternative_is_inlier;
                R_C_W_guess(:, :, 1) = R_C_W_guess(:, :, 2);
                t_C_W_guess(:,:,1) = t_C_W_guess(:,:,2);
            end
        end

        if tweaked_for_more
            min_inlier_count = 30;
        else
            min_inlier_count = 6;
        end

        if nnz(is_inlier) > max_num_inliers && ...
                nnz(is_inlier) >= min_inlier_count
            max_num_inliers = nnz(is_inlier);        
            best_inlier_mask = is_inlier;
            best_guess = [R_C_W_guess(:, :, 1), t_C_W_guess(:, :, 1)];
            best_guess_WC = [R_C_W_guess(:, :, 1)', -R_C_W_guess(:, :, 1)' * t_C_W_guess(:, :, 1)];
        end

        max_num_inliers_history(i) = max_num_inliers;
    end

    if max_num_inliers == 0
        T_WC = [];
        T_CW = [];
        best_inlier_mask = [];
    else
        T_WC = best_guess_WC;
        T_CW = best_guess;
    end

end

