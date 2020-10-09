clear all
close all
clc
init_workspace
%% Define dataset path
kitti_path = 'data/kitti';
malaga_path = 'data/malaga-urban-dataset-extract-07';
parking_path = 'data/parking';
grocery_path = 'data/grocery/960x540';
%% Setup
global ds;
ds = 0; % 0: KITTI, 1: Malaga, 2: parking, 3: grocery
save_video = false;

global K debug matlabfuncs reprojection_error_threshold slack_radius;
global angle_threshold min_quality gauss_filter_size cameraParams;
global harris_patch_size num_keypoints harris_kappa nonmaximum_supression_radius;
global candidate_threshold landmark_upperbound;
global cart_mask cart_sigma;
rng(2);

debug = false;
matlabfuncs = true; % to use matlab implementations instead of own algos
reprojection_error_threshold = 100;
landmark_upperbound = Inf;
slack_radius = 0.2;
angle_threshold = 0.8;
min_quality = 1e-2;
gauss_filter_size = 5;
candidate_threshold = 5;

harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 200;
nonmaximum_supression_radius = 20;

if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    images = dir([kitti_path '/00/image_0']);
    images = images(3:end);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = length(images);
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    num_keypoints = 200;
    nonmaximum_supression_radius = 30;
    angle_threshold = 0.8;
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    images = images(3:2:end);
    last_frame = length(images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    images = dir([parking_path '/images']);
    images = images(4:end); % start from idx 4 to skip k.txt under image folder
    last_frame = length(images);
    K = load([parking_path '/K.txt']);
    
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    min_quality = 1e-5;
elseif ds == 3
    assert(exist('grocery_path', 'var') ~= 0);
    images = dir(grocery_path);
    images = images(5:end);  % Ignore other files in the directory
    images = images(100:650); % Limit the number of frames used for testing
    last_frame = length(images);
    % Get the undistorted image intrinsics to find K
    min_quality = 1e-5;
    K = load(fullfile(grocery_path,'K.txt'));
    %num_keypoints = 1000;
    nonmaximum_supression_radius = 10;
    %angle_threshold = 1.5;
    load(fullfile('data/cart_mask.mat'));
    cart_sigma = 10;
else
    assert(false);
end

cameraParams = cameraParameters('IntrinsicMatrix', K');

%% Bootstrap
% need to set bootstrap_frames
bootstrap_frames = [1,2,3];

if ds == 0 % KITTI dataset with grayscale images
    img0 = imread(fullfile(images(bootstrap_frames(1)).folder, images(bootstrap_frames(1)).name));
    img1 = imread(fullfile(images(bootstrap_frames(2)).folder, images(bootstrap_frames(2)).name));
    img2 = imread(fullfile(images(bootstrap_frames(3)).folder, images(bootstrap_frames(3)).name));
else % color images, convert to grayscale
    img0 = rgb2gray(imread(fullfile(images(bootstrap_frames(1)).folder, images(bootstrap_frames(1)).name)));
    img1 = rgb2gray(imread(fullfile(images(bootstrap_frames(2)).folder, images(bootstrap_frames(2)).name)));
    img2 = rgb2gray(imread(fullfile(images(bootstrap_frames(3)).folder, images(bootstrap_frames(3)).name)));
end

%% INITIALIZATION: STRUCTURE FROM MOTION

[matchedPoints1, matchedPoints2] = getInitialCorrespondances(img0, img1, img2);

[keypoints, p_W_landmarks, matchedPoints1, matchedPoints2, T_WC_Bootstrap] = ...
    runSFM(matchedPoints1, matchedPoints2);

T_WC_Bootstrap = homogenizePose(T_WC_Bootstrap);
p_W_landmarks = (T_WC_Bootstrap\[p_W_landmarks, ones(size(p_W_landmarks,1),1)]')';
p_W_landmarks = p_W_landmarks(:, 1:3);

%% CONTINUOUS OPERATION

% % UNCOMMENT TO USE GROUND TRUTH
% % use ground truth provided in exercise 7 (000000.png, left camera, KITTI)
% p_W_landmarks = load('data/p_W_landmarks.txt');
% keypoints = load('data/keypoints.txt');
% keypoints = [keypoints(:, 2), keypoints(:, 1)];
% bootstrap_frames(1) = 0;

S0.idx = bootstrap_frames(end);
S0.path = fullfile(images(S0.idx).folder, images(S0.idx).name);
S0.frame = imread(S0.path);
if (size(S0.frame, 3) == 3)
    S0.frame = rgb2gray(S0.frame);
end
S0.keypoints = keypoints;
S0.landmarks = p_W_landmarks;
S0.c_keypoints = [];
S0.f_c_keypoints = [];
S0.t_c_keypoints = [];
S0.T_WC = eye(3,4);


%% Test single frame
%[S1, T_WC_current] = processFrame(S0, S0.idx + 1, images);

frame_idx = (bootstrap_frames(end)+1):last_frame;
trajectory = []; num_landmarks_history = [];
S_previous = S0;
if ~save_video
    figure(1);
else
    figure('Position',[0,0,1360,760]);
end
trajectory_R = [];
trajectory_t = [];
last_time_initialized = 0;

if save_video
    results_vid = VideoWriter('results.avi');
    results_vid.FrameRate = 10;
    open(results_vid);
end
for idx = frame_idx
    fprintf('\n Processing frame number  : %d \n', idx);
    [S_current, T_WC_current] = processFrame(S_previous, idx, images);
    
    trajectory = [trajectory, T_WC_current(:, 4)]; 
    num_landmarks_history = [num_landmarks_history, size(S_current.landmarks, 1)];
    
        
    figure(1);
    subplot(2, 4, [1,2]);
    imshow(S_current.frame); hold on;
    title('Current Frame');
    scatter(S_current.c_keypoints(:,1), S_current.c_keypoints(:,2), 'r.'); hold on;
    scatter(S_current.keypoints(:, 1), S_current.keypoints(:, 2), 'g.'); hold off;
    
    % plot # of landmarks tracking over last 20 frames
    subplot(2, 4, 5); 
    idx_plt = max(length(num_landmarks_history) - 19, 1);
    plot(-(length(num_landmarks_history)-idx_plt):0, num_landmarks_history(idx_plt:end));
    title('# tracked landmarks over last 20 frames');
    
    % plot entire trajectory
    subplot(2, 4, 6); 
    plot(trajectory(1, :), trajectory(3, :), 'x-'); 
    title('Full Trajectory');
    axis equal;
    
    % plot trajectory of last 20 frames and landmarks
    subplot(2, 4, [3,4,7,8]);
    scatter(S_current.landmarks(:, 1), S_current.landmarks(:, 3), 'k.'); hold on;
    %scatter(p_W_landmarks(:, 1), p_W_landmarks(:, 3), 'g.'); hold on; 
    idx_plt = max(1, size(trajectory, 2)-19):size(trajectory, 2);
    plot(trajectory(1, idx_plt), trajectory(3, idx_plt), 'x-'); hold off;
    title('Trajectory of last 20 frames and landmarks');
    centre = [trajectory(1, end), trajectory(3, end)];
    scale = max(range(trajectory(1, idx_plt)), range(trajectory(3, idx_plt)))+eps;
    axis([trajectory(1, end)-scale, ... 
          trajectory(1, end)+scale, ...
          trajectory(3, end)-scale, ... 
          trajectory(3, end)+scale]);

    if save_video
        results_frame = getframe(gcf);
        writeVideo(results_vid, results_frame);
    end
%     if norm(S_current.T_WC(1:3, 4) - S_previous.T_WC(1:3, 4)) < 0.05 & ...
%             (idx - last_time_initialized) > 20
%         
%         fprintf('Reinitializing the landmarks\n\n');
%         last_time_initialized = idx;
%         img0 = imread(fullfile(images(idx-2).folder, images(idx-2).name));
%         [matchedPoints1, matchedPoints2] = getInitialCorrespondances(img0, S_previous.frame, S_current.frame);
%         [S_current.keypoints, S_current.landmarks, matchedPoints1, matchedPoints2, T_WC_Bootstrap] = ...
%         runSFM(matchedPoints1, matchedPoints2);
%         S_current.T_WC = homogenizePose(S_previous.T_WC)*homogenizePose(T_WC_Bootstrap);
%         S_current.landmarks = S_current.T_WC*[S_current.landmarks, ones(size(S_current.landmarks, 1), 1)]';
%         S_current.landmarks = S_current.landmarks(1:3, :)';
%         S_current.T_WC = S_current.T_WC(1:3, :);
%         S0.c_keypoints = [];
%         S0.f_c_keypoints = [];
%         S0.t_c_keypoints = [];
%         
%     end
    S_previous = S_current;
    if ~save_video
        pause(0.1);
    else
        drawnow;
    end
    %keyboard;
end

if save_video
    close(results_vid);
end
