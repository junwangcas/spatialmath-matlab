% landmark fix, pose optimize
clear;
DoF = 3;
Dim = 2;

NUM_POSES = 1;
NUM_LMKS = 5;
NUM_FACTORS = 5;
NUM_STATES = NUM_POSES * DoF;
NUM_MEAS = NUM_POSES * NUM_LMKS * Dim;
MAX_ITER = 20;

% Define five landmarks (beacons) in R^2
landmarks_simu = [];
landmarks_simu = [landmarks_simu;[3.0,  0.0]];
landmarks_simu = [landmarks_simu;[2.0, -1.0]];
landmarks_simu = [landmarks_simu;[2.0,  1.0]];
landmarks_simu = [landmarks_simu;[3.0, -1.0]];
landmarks_simu = [landmarks_simu;[3.0,  1.0]];

% Define the beacon's measurements in R^2
y_sigmas = [0.001, 0.001]';
S = eye(2, 2);

pairs = [];  % pose - landmark
pairs = [pairs; [1, 1]];
pairs = [pairs; [1, 2]];
pairs = [pairs; [1, 3]];
pairs = [pairs; [1, 4]];
pairs = [pairs; [1, 5]];


% Simulator
% poses, controls
poses_simu = cell(NUM_POSES, 1);
poses = cell(NUM_POSES, 1);

poses_simu{1} = SE2(1, 2, 0.7854);
poses{1} = poses_simu{1} * SE2.rand;

% measures
measurements = zeros(length(pairs), 2);
for id_pair = 1:length(pairs)
    id_pose = pairs(id_pair, 1);
    id_lmk = pairs(id_pair, 2);
    X_simu = poses_simu{id_pose};
    Xi = poses{id_pose};
    
    b = landmarks_simu(id_lmk, :)';
    y_noise = y_sigmas * rand();
    y = X_simu.inv * b;
    
    measurements(id_pair, :) = y; % + y_noise;
end

%% visualize
% ground truth 
figure();
plot(landmarks_simu(:,1), landmarks_simu(:,2),'*r');
for i = 1:NUM_POSES
    hold on; poses_simu{i}.plot;
end
xlim([-1, 4]); ylim([-2, 3]);
axis auto
title('1lmks poses - gt');
% initial values
figure();
plot(landmarks_simu(:,1), landmarks_simu(:,2),'*r');
for i = 1:NUM_POSES
    hold on; poses{i}.plot;
end
xlim([-1, 4]); ylim([-2, 3]);
%axis equal
axis auto
title('2lmks poses - init');

% estimator
for iteration = 1:MAX_ITER
    r = zeros(NUM_MEAS, 1);
    J = zeros(NUM_MEAS, NUM_STATES);
    row = 1;
    
    for i = 1 : length(pairs)
        id_lmk = pairs(i, 2);
        X = poses{1};
        b = landmarks_simu(i, :);
        y = measurements(i, :);
        
        e = X.inv * b';
        r(row:(row + Dim - 1), 1) = S * (e - y');
        
        cols = 1 : 3; 
        ix = X.inv;
        J_e_ix = [ix.R, ix.R*skew(1)*b']; % equation 166
        J_ix_x = zeros(3, 3);  % equat 160 
        J_ix_x(1:2, 1:2) = -X.R;
        J_ix_x(1:2, 3) = skew(1) * X.transl';
        J_ix_x(3, 3) = -1;
        J(row:(row + Dim - 1), cols) = J_e_ix * J_ix_x;  % chain rule
        
        row = row + Dim;
    end
    
    dx = - inv(J' * J) * J' * r
    % update
    poses{1} = poses{1} * SE2.exp(vec2se2([dx()]));
    
    % plot
    figure(3);
    clf;
    plot(landmarks_simu(:,1), landmarks_simu(:,2),'*r');
    for i = 1:NUM_POSES
        hold on; poses{i}.plot;
    end
    xlim([-1, 4]); ylim([-2, 3]);
    %axis equal
    axis auto
    title('3lmks poses - optimization');
    waitforbuttonpress;
end































