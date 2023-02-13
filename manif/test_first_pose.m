% only have the first pose
clear;
DoF = 3;
Dim = 2;

NUM_POSES = 1;
NUM_LMKS = 0;
NUM_FACTORS = 0;
NUM_STATES = NUM_POSES * DoF + NUM_LMKS * Dim;
NUM_MEAS = NUM_POSES * DoF + NUM_FACTORS * Dim;
MAX_ITER = 5;


% Simulator
% poses, controls
poses_simu = cell(NUM_POSES, 1);
poses = cell(NUM_POSES, 1);
controls = cell(NUM_POSES, 1);

poses_simu{1} = SE2(10, 20, 1.3);
poses{1} = poses_simu{1} * SE2(100, 10, 1.5);


%% visualize
% ground truth 
subplot(1, 3, 1);
for i = 1:NUM_POSES
    hold on; poses_simu{i}.plot;
end
xlim([-1, 4]); ylim([-2, 3]);
axis auto
title('1 poses - gt');
% initial values
subplot(1, 3, 2);
for i = 1:NUM_POSES
    hold on; poses{i}.plot;
end
xlim([-1, 4]); ylim([-2, 3]);
%axis equal
axis auto
title('2 poses - init');

% estimator
sploth = subplot(1, 3, 3);
for iteration = 1:MAX_ITER
    r = zeros(NUM_MEAS, 1);
    J = zeros(NUM_MEAS, NUM_MEAS);
    row = 1;
    col = 1;
    
    % first pose
    delta_pose = poses_simu{1}.inv * poses{1};
    r(row:(row + DoF - 1), 1) = se2vec(delta_pose.log);
    J(row:(row + DoF - 1), col:(col + DoF - 1)) = eye(3,3); % ref eq: 162
    
    dx = - inv(J' * J) * J' * r
    % update
    poses{1} = poses{1} * SE2.exp(vec2se2([dx()]));
    
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































