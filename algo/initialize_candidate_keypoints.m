function [c_keypoints, f_c_keypoints, t_c_keypoints] = ...
    initialize_candidate_keypoints(frame, current_kp, T_WC)
    global matlabfuncs;
    global min_quality;
    global gauss_filter_size;
    global candidate_threshold;
    
    debug = false;
    
    if matlabfuncs
        candidate_kp = customHarris(frame);
%         points = detectHarrisFeatures(frame, ...
%             'MinQuality', min_quality, ...
%             'FilterSize', gauss_filter_size);
% 
%         [~, candidate_kp] = extractFeatures(frame,points);
%         candidate_kp = candidate_kp.Location;
%         
%         candidate_kp = removeClose2DPoints(...
%                     candidate_kp, current_kp, candidate_threshold);
    else
        candidate_kp = customHarris(frame);
    end
   
   if debug
       figure;
       imshow(frame); hold on;
       scatter(current_kp(:,1), current_kp(:,2),'xr');
       scatter(candidate_kp(:,1), candidate_kp(:,2),'xg');
       hold off; 
   end
   
   c_keypoints = candidate_kp;
   f_c_keypoints = candidate_kp;
   t_c_keypoints = repmat(reshape(T_WC,1,12), size(candidate_kp, 1), 1);
end