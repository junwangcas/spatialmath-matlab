clear;
DoF = 3;

NUM_POSES = 2;
NUM_STATES = NUM_POSES * DoF;
NUM_MEAS = NUM_POSES * DoF;
MAX_ITER = 20;


% Define a control vector and its noise and covariance in the tangent of SE2
u = [0, 0, 0]';          % control signal, generic
u_nom = [1.0, 0.0, 0.5]';      % nominal control signal
u_sigmas = [0.0, 0.0, 0.0]';   % control noise std specification
%u_noise = 0;    % control noise
%W = inv(diag(u_sigmas));          % sqrt Info
W = eye(3, 3);
controls = [];   % robot controls


% Simulator
% poses, controls
poses_simu = cell(NUM_POSES, 1);
poses = cell(NUM_POSES, 1);
controls = cell(NUM_POSES, 1);

poses_simu{1} = SE2(1, 2, 0.7854);
poses{1} = poses_simu{1} * SE2.rand;
for id_pose = 2:NUM_POSES
    X_simu = poses_simu{id_pose -1} * SE2.exp(vec2se2(u_nom));
    
    u_noise = u_sigmas * rand;
    X_i = poses{id_pose - 1} * SE2.exp(vec2se2(u_nom + u_noise));
    
    poses_simu{id_pose} = X_simu;
    poses{id_pose} = X_i * SE2.exp(vec2se2(rand(3, 1)));
    
    controls{id_pose} = u_nom + u_noise;
end

%% visualize
% ground truth 
subplot(1, 3, 1);
for i = 1:NUM_POSES
    hold on; poses_simu{i}.plot;
end
xlim([-1, 4]); ylim([-2, 3]);
axis auto
title('1lmks poses - gt');
% initial values
subplot(1, 3, 2);
for i = 1:NUM_POSES
    hold on; poses{i}.plot;
end
xlim([-1, 4]); ylim([-2, 3]);
%axis equal
axis auto
title('2lmks poses - init');

% estimator
sploth = subplot(1, 3, 3);
for iteration = 1:MAX_ITER
    r = zeros(NUM_MEAS, 1);
    J = zeros(NUM_MEAS, NUM_STATES);
    row = 1;
    
    % first pose
    delta_pose = poses_simu{1}.inv * poses{1};
    r(row:(row + DoF - 1), 1) = se2vec(delta_pose.log);
    J(row:(row + DoF - 1), 1:3) = eye(3,3); % ref eq: 41c
    
    row = row + DoF;
    % evaluate motion factors
    for i = 1:NUM_POSES-1
        j = i + 1;
        Xi = poses{i};
        Xj = poses{j};
        u = controls{j};
        
        delta_pose = (Xi.inv * Xj);
        d = se2vec(delta_pose.log);
        r(row : (row + DoF -1), 1) = W * (d - u);
        J_d_xj = eye(3, 3); % 162
        
        
        ix = Xi.inv;  % 161
        J_d_ixi = eye(3, 3);
        J_d_ixi(1:2, 1:2) = Xj.R';
        J_d_ixi(1:2, 3) = Xj.R' * skew(1) * Xj.transl';
        
        J_ix_x = zeros(3, 3);  % equat 160 
        J_ix_x(1:2, 1:2) = Xi.R;
        J_ix_x(1:2, 3) = skew(1) * Xi.transl';
        J_ix_x(3, 3) = -1;
        
        J_d_xi = J_d_ixi * J_ix_x; 
        
        colsi = ((i - 1) * DoF + 1):((i - 1) * DoF + 3);
        J(row : (row + DoF -1), colsi) = J_d_xi;
        colsj = ((j - 1) * DoF + 1):((j - 1) * DoF + 3);
        J(row : (row + DoF -1), colsj) = J_d_xj;
    end
    
    dx = - inv(J' * J) * J' * r
    % update
    poses{1} = poses{1} * SE2.exp(vec2se2([dx()]));
    
    % plot
    %clf;
    for i = 1:NUM_POSES
        hold on; poses{i}.plot;
    end
    xlim([-1, 4]); ylim([-2, 3]);
    %axis equal
    axis auto
    title('3 poses - optimization');
    waitforbuttonpress;
    cla(sploth);
    
end































