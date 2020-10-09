function [new_points_2d_W, new_points_3d_W] = triangulate_new_landmarks(S_current, ...
                                                                        c_keypoints, ...
                                                                        t_c_keypoints, ...
                                                                        f_c_keypoints, ...
                                                                        T_W_C)
    global K;
    global landmark_upperbound;
    
    num_of_c_kps = size(c_keypoints, 1);
    W_points = zeros(num_of_c_kps, 3);
    
    T_W_C_current = homogenizePose(T_W_C);
    T_C_current_W = inv(T_W_C_current);
    
    for i = 1:num_of_c_kps

         old_pose = reshape(t_c_keypoints(i, :), 3, 4);            
         T_W_C_prev = homogenizePose(old_pose); 
         T_C_prev_W = inv(T_W_C_prev); 

        [W_point] = ...
            linearTriangulation([c_keypoints(i, :), 1]', ...
                        [f_c_keypoints(i, :), 1]', ...
                        K*T_C_current_W(1:3,:),...
                        K*T_C_prev_W(1:3,:));

        W_points(i, :) =  W_point(1:3)';
    end
   
    % Just check if infront of camera
    W_points = T_W_C_current\([W_points ones(size(W_points,1),1)]');
    
    valid_points = (W_points(3,:)' > 0) & (W_points(3,:)' < landmark_upperbound);
    fprintf("behind camera: %d \n", sum(W_points(3,:)' < 0));
    
    W_points = W_points(:, valid_points);
    
    new_points_3d_W = T_W_C_current*W_points;
    new_points_3d_W = new_points_3d_W(1:3,:)';
    
    fprintf('Final valid points to triangulate are  : %d \n', size(new_points_3d_W, 1));
    new_points_2d_W = c_keypoints(valid_points, :);
end