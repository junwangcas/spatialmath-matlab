clear;
DoF = 3;
Dim = 2;

NUM_POSES = 3;
NUM_LMKS = 5;
NUM_FACTORS = 9;
NUM_STATES = NUM_POSES * DoF + NUM_LMKS * Dim;
NUM_MEAS = NUM_POSES * DoF + NUM_FACTORS * Dim;
MAX_ITER = 20;


% Define a control vector and its noise and covariance in the tangent of SE2
u = [0, 0, 0]';          % control signal, generic
u_nom = [1.0, 0.0, 0.5]';      % nominal control signal
u_sigmas = [0.01, 0.01, 0.01]';   % control noise std specification
%u_noise = 0;    % control noise
W = inv(diag(u_sigmas));          % sqrt Info
controls = [];   % robot controls

% Define five landmarks (beacons) in R^2
landmarks_simu = [];
landmarks_simu = [landmarks_simu;[3.0,  0.0]];
landmarks_simu = [landmarks_simu;[2.0, -1.0]];
landmarks_simu = [landmarks_simu;[2.0,  1.0]];
landmarks_simu = [landmarks_simu;[3.0, -1.0]];
landmarks_simu = [landmarks_simu;[3.0,  1.0]];

% Define the beacon's measurements in R^2
y_sigmas = [0.001, 0.001]';
S = inv(diag(y_sigmas));

pairs = [];  % pose - landmark
pairs = [pairs; [1, 1]];
pairs = [pairs; [1, 2]];
pairs = [pairs; [1, 4]];
pairs = [pairs; [2, 1]];
pairs = [pairs; [2, 3]];
pairs = [pairs; [2, 5]];
pairs = [pairs; [3, 2]];
pairs = [pairs; [3, 3]];
pairs = [pairs; [3, 5]];

% Simulator
% poses, controls
poses_simu = cell(NUM_POSES, 1);
poses = cell(NUM_POSES, 1);
controls = cell(NUM_POSES, 1);

poses_simu{1} = SE2;
poses{1} = SE2 * SE2.rand;
for id_pose = 2:NUM_POSES
    X_simu = poses_simu{id_pose -1} * SE2.exp(vec2se2(u_nom));
    
    u_noise = u_sigmas * rand;
    X_i = poses{id_pose - 1} * SE2.exp(vec2se2(u_nom + u_noise));
    
    poses_simu{id_pose} = X_simu;
    poses{id_pose} = X_i * SE2.exp(vec2se2(rand(3, 1)));
    
    controls{id_pose} = u_nom + u_noise;
end

% measures
measurements = zeros(length(pairs), 2);
landmarks = zeros(length(landmarks_simu), 2);
for id_pair = 1:length(pairs)
    id_pose = pairs(id_pair, 1);
    id_lmk = pairs(id_pair, 2);
    X_simu = poses_simu{id_pose};
    Xi = poses{id_pose};
    
    b = landmarks_simu(id_lmk, :)';
    y_noise = y_sigmas * rand();
    y = X_simu.inv * b;
    
    measurements(id_pair, :) = y + y_noise;
    b = Xi * (y + y_noise);
    landmarks(id_lmk, :) = b; 
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
plot(landmarks(:,1), landmarks(:,2),'*r');
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
    col = 1;
    
    % first pose
    r(row:row + DoF - 1, 1) = se2vec(poses{1}.log);
    J(row:row + DoF - 1, col:col+DoF) = eye(3,3); % ref eq: 41c
    
    row = row + DoF;
    % evaluate motion factors
    for i = 1:NUM_POSES-1
        j = i + 1;
        Xi = poses{i};
        Xj = poses{j};
        u = controls{j};
        
        delta_pose = (Xi.inv * Xj);
        d = delta_pose.log;
        r(row : row + DoF -1, 1) = W * (d - u);
        J_d_xj = eye(3, 3); % 66 
        %J_d_xi = % eq 62.
    end
    
end































