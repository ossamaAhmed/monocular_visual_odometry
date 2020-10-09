function valid_mask = is_trianguable(c_kp, current_pose, f_c_kp, t_c_kp)
    global angle_threshold
    debug = false;
    
    current_pose = [current_pose; 0 0 0 1];
    
    W_bearing_c_kp = computeBearing(c_kp, eye(3,4));
    W_bearing_f_c_kp = zeros(size(t_c_kp, 1), 3);
    
    for i = 1:size(f_c_kp,1)
        pose = reshape(t_c_kp(i, :), 3, 4);
        pose = [pose; 0 0 0 1];
        relative_pose = inv(pose)\current_pose;
        relative_pose = inv(relative_pose);
        
        W_bearing_f_c_kp(i, :) = computeBearing(f_c_kp(i, :), relative_pose);
    end
    
    angles = zeros(size(W_bearing_c_kp,1), 1);
    
    for i = 1:size(W_bearing_c_kp, 1)
        angles(i) = abs(atan2d(norm(cross(W_bearing_c_kp(i, :)', W_bearing_f_c_kp(i, :)')), ...
            dot(W_bearing_c_kp(i, :)', W_bearing_f_c_kp(i, :)')));
    end
    
    valid_mask = angles > angle_threshold;
    if debug 
        fprintf('min triangulated angle %d : \n', min(angles(angles > angle_threshold)));
        fprintf('num triangulable points %d : \n', sum(angles > angle_threshold));
    end

end

function W_bearings = computeBearing(kps, T_WC)
    global K;
    C_bearings = K\([kps, ones(size(kps,1), 1)])';
    R_WC = T_WC(1:3, 1:3);
    W_bearings = R_WC*C_bearings;
    W_bearings = W_bearings';
end