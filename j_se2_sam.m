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
plot(landmarks_simu(:,1), landmarks_simu(:,2),'*');
%xlim([-5, 5]); ylim([-5, 5]);
axis equal
title('1landmarks - gt');

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
%visualize 
for i = 1:NUM_POSES
    hold on; poses{i}.plot;
    if i == 3
        t = poses{i}
        t.plot;
        return
    end
end
%hold on; X_i.plot;
%hold on; X_simu.plot;

% measures

measurements = zeros(length(pairs), 2);
landmarks = zeros(length(landmarks_simu), 2);
for id_pair = 1:length(pairs)
    id_pose = pairs(id_pair, 1);
    id_lmk = pairs(id_pair, 2);
    
    b = landmarks_simu(id_lmk, :)';
    y_noise = y_sigmas * rand();
    y = X_simu.inv * b;
    
    measurements(id_pair, :) = y + y_noise;
    b = Xi * (y + y_noise);
    landmarks(id_lmk, :) = b; 
end



























