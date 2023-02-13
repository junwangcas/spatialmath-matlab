clear;
% 固定pose，lmk为变量
Dim = 2;

NUM_POSES = 1;
NUM_LMKS = 5;
NUM_FACTORS = 5;
NUM_STATES = NUM_LMKS * Dim;
NUM_MEAS = NUM_FACTORS * Dim;
MAX_ITER = 20;

% Define five landmarks (beacons) in R^2
landmarks_simu = [];
landmarks_simu = [landmarks_simu;[3.0,  0.0]];
landmarks_simu = [landmarks_simu;[2.0, -1.0]];
landmarks_simu = [landmarks_simu;[2.0,  1.0]];
landmarks_simu = [landmarks_simu;[3.0, -1.0]];
landmarks_simu = [landmarks_simu;[3.0,  1.0]];

% Define the beacon's measurements in R^2
y_sigmas = [0.01, 0.01]';
y_sigmas = [0.0, 0.0]';
S = inv(diag(y_sigmas));
S = eye(2, 2);

pairs = [];  % pose - landmark
pairs = [pairs; [1, 1]];
pairs = [pairs; [1, 2]];
pairs = [pairs; [1, 3]];
pairs = [pairs; [1, 4]];
pairs = [pairs; [1, 5]];


% Simulator
poses_simu = cell(NUM_POSES, 1);
poses_simu{1} = SE2 * SE2.exp(vec2se2([1, 2, 0.5]));
% measures
measurements = zeros(length(pairs), 2);
landmarks = zeros(length(landmarks_simu), 2);
for id_pair = 1:length(pairs)
    id_pose = pairs(id_pair, 1);
    id_lmk = pairs(id_pair, 2);
    X_simu = poses_simu{id_pose};
    
    b = landmarks_simu(id_lmk, :)';
    y_noise = y_sigmas * rand();
    y = X_simu.inv * b;
    
    measurements(id_pair, :) = y + y_noise;
    landmarks(id_lmk, :) = b + rand(2, 1);%*100; 
end

%% visualize
% ground truth 
subplot(1, 3, 1);
plot(landmarks_simu(:,1), landmarks_simu(:,2),'*r');
hold on; poses_simu{1}.plot;
xlim([-1, 4]); ylim([-2, 3]);
axis auto
title('1lmks poses - gt');
% initial values
subplot(1, 3, 2);
plot(landmarks(:,1), landmarks(:,2),'*r');
for i = 1:NUM_POSES
    hold on; poses_simu{i}.plot;
end
xlim([-1, 4]); ylim([-2, 3]);
%axis equal
axis auto
title('2lmks poses - init');

sploth = subplot(1, 3, 3);
% estimator
for iteration = 1:MAX_ITER
    iteration
    r = zeros(NUM_MEAS, 1);
    J = zeros(NUM_MEAS, NUM_STATES);
    row = 1;
    
    for i = 1 : length(pairs)
        id_lmk = pairs(i, 2);
        X = poses_simu{1};
        b = landmarks(i, :);
        y = measurements(i, :);
        
        e = X.inv * b';
        r(row:(row + Dim - 1), 1) = S * (e - y');
        
        cols = (2 * id_lmk - 1) : (2 * id_lmk);
        %J(row:(row + Dim - 1), cols) = X.inv * eye(2, 2);
        %J(row:(row + Dim - 1), cols) = X.R;
        %ix = X.inv;
        %J_e_ix = [ix.R, ix.R*skew(1)*b'];
        %J_ix_x = zeros(3, 3);
        %J_ix_x(1:2, 1:2) = -X.R;
        %J_ix_x(1:2, 3) = skew(1) * X.transl;
        %J(row:(row + Dim - 1), cols) = J_e_ix * J_ix_x;
        ix = X.inv;
        J_e_p = ix.R;
        J(row:(row + Dim - 1), cols) = J_e_p; %% equ 167
        
        
        
        row = row + Dim;
    end
    
    dx = - inv(J' * J) * J' * r
    % update
    for i = 1 : length(landmarks)
        landmarks(i, :) = landmarks(i, :) + dx((Dim * i - 1) : (Dim * i))';
    end
    
    % plot
    plot(landmarks(:,1), landmarks(:,2),'*r');
    for i = 1:NUM_POSES
        hold on; poses_simu{i}.plot;
    end
    xlim([-1, 4]); ylim([-2, 3]);
    %axis equal
    axis auto
    title('3lmks poses - optimization');
    waitforbuttonpress;
    cla(sploth);
end































