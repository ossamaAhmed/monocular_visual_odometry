function [S_current, T_WC] = processFrame(S_previous, idx_next, imgs_idx)

    global K;
    global slack_radius;
    global cameraParams;
    global matlabfuncs;
    global ds;
    global cart_mask cart_sigma;
    debug = false;
    
    S_current.idx = idx_next;
    S_current.path = fullfile(imgs_idx(idx_next).folder, imgs_idx(idx_next).name);
    curr_frame = imread(S_current.path);
    if (size(curr_frame, 3) == 3)
        curr_frame = rgb2gray(curr_frame);
    end
    if ds==3
        % Blur the region of the shoping cart to reduce tracking errors
        curr_frame = roifilt2(fspecial('gaussian',cart_sigma*3,cart_sigma),curr_frame,cart_mask);
    end
    S_current.frame = curr_frame;
    S_current.c_keypoints = [];
    S_current.c_keypoints_old = S_previous.c_keypoints;
    S_current.f_c_keypoints = [];
    S_current.t_c_keypoints = [];
    S_current.keypoints = [];
    S_current.landmarks = [];
    S_current.T_WC = [];
    %track keypoints between last state and the current state
    

    
    [matched_keypoints, valid_mask] = track_points_between_frames(S_previous.keypoints, ...
                                                                  S_previous.frame, ...
                                                                  S_current.frame);
    matched_landmarks = S_previous.landmarks(valid_mask, :);
    
    if matlabfuncs        
        [R, t, best_inlier_mask] = estimateWorldCameraPose(matched_keypoints, ...
                                                           matched_landmarks, ...
                                                           cameraParams, ...
                                                           'MaxNumTrials', 1000, ...
                                                           'Confidence', 99, ...
                                                           'MaxReprojectionError', 6);
        
        T_WC = [R', t'];
        
       
        
    else
        % Gives the transpose of the rotation matrix Potential Bug
        [T_WC, T_CW, best_inlier_mask, ~] = ransacLocalization(matched_keypoints, ...
                                                          matched_landmarks, K);
    end
    
    if debug && (S_current.idx >= 9)
        %figure(7);
        %showMatchedFeatures(S_current.frame, S_current.frame, c_keypoints, f_c_keypoints); hold on;
        %figure(8);
        hom_T_WC = homogenizePose(T_WC);
        hom_T_WC_prev = homogenizePose(S_previous.T_WC);
        T_C_prev_C = hom_T_WC_prev\hom_T_WC;
        %scatter(C_curr_points(:,1), C_curr_points(:,3), 'r.'); hold off;
        %scatter(new_points_3d(1:200,1), new_points_3d(1:200,3), 'b.'); hold on;
        %axis([-20,20,-10,50])
    end
    
    
    %fprintf('matched keypointss size  : %d \n', size(matched_keypoints,1));                                            
    %fprintf('best_inlier_mask is  : %d \n', sum(best_inlier_mask));
    %tracked features between frame i and frame i+1
    tracked_landmarks = matched_landmarks(best_inlier_mask, :);
    tracked_keypoints = matched_keypoints(best_inlier_mask, :);
    
    T_WC = pose_refinement(T_WC, tracked_keypoints, tracked_landmarks, cameraParams);
        
    %track candidiate keypoints from last state and the current state
    if isempty(S_previous.c_keypoints)
        [S_current.c_keypoints, S_current.f_c_keypoints, S_current.t_c_keypoints] = ...
            initialize_candidate_keypoints(S_current.frame, ...
                                           tracked_keypoints, ...
                                           T_WC);
        
        
        S_current.landmarks = tracked_landmarks;
        S_current.keypoints = tracked_keypoints;
    else
        [matched_c_keypoints, valid_mask] = track_points_between_frames(S_previous.c_keypoints, ...
                                                                        S_previous.frame, ...
                                                                        S_current.frame);
        matched_f_c_keypoints = S_previous.f_c_keypoints(valid_mask, :); 
        matched_t_c_keypoints = S_previous.t_c_keypoints(valid_mask, :);
        
        %check which ones from the tracked ones are triangulable
        valid_mask_t = is_trianguable(matched_c_keypoints, ...
                                      T_WC, ...
                                      matched_f_c_keypoints, ...
                                      matched_t_c_keypoints);
                                  
               
        triangulable_c_keypoints = matched_c_keypoints(valid_mask_t, :);
        triangulable_t_c_keypoints = matched_t_c_keypoints(valid_mask_t, :);
        triangulable_f_c_keypoints = matched_f_c_keypoints(valid_mask_t, :);
        
        
        %figure(8); showMatchedFeatures(S_current.frame, S_current.frame, triangulable_c_keypoints, triangulable_f_c_keypoints); hold off;

        %now triangulate them
        [new_points_2d_world, new_points_3d_world] = triangulate_new_landmarks(S_current, ...
                                                                               triangulable_c_keypoints, ...
                                                                               triangulable_t_c_keypoints, ...
                                                                               triangulable_f_c_keypoints, ...
                                                                               T_WC);
         new_keypoints = [tracked_keypoints;  new_points_2d_world];
         new_landmarks = [tracked_landmarks; new_points_3d_world];
         %now the ones I didn't triangulate we should save them as
        %potential candidates
         non_triangulated_ckps = matched_c_keypoints(valid_mask_t == 0, :);
         non_triangulated_t_ckps = matched_t_c_keypoints(valid_mask_t == 0, :);
         non_triangulated_f_ckps = matched_f_c_keypoints(valid_mask_t == 0, :);
        [S_current.c_keypoints, ~, ~] = ...
                        initialize_candidate_keypoints(S_current.frame, ...
                                                       tracked_keypoints, ...
                                                       T_WC);
                                                   
                                                   
        S_current.c_keypoints = removeClose2DPoints(S_current.c_keypoints, matched_c_keypoints, slack_radius);
        S_current.f_c_keypoints = S_current.c_keypoints;
        S_current.t_c_keypoints = repmat(reshape(T_WC,1,12), size(S_current.f_c_keypoints, 1), 1);

        S_current.c_keypoints = [non_triangulated_ckps; S_current.c_keypoints];
        S_current.f_c_keypoints = [non_triangulated_f_ckps; S_current.f_c_keypoints];
        S_current.t_c_keypoints = [non_triangulated_t_ckps; S_current.t_c_keypoints];
        S_current.landmarks = new_landmarks;
        S_current.keypoints = new_keypoints;
     end
    
    S_current.T_WC = T_WC;
    
%     disp(['Num of key points for next state is : ' num2str(size(S_next.landmarks,1))]);
 end



