% landmark
clear;
DoF = 3;

NUM_POSES = 1;
NUM_STATES = NUM_POSES * DoF;
NUM_MEAS = NUM_POSES * DoF;
MAX_ITER = 20;

% Simulator
% poses, controls
poses_simu = cell(NUM_POSES, 1);
poses = cell(NUM_POSES, 1);

poses_simu{1} = SE2(1, 2, 0.7854);
poses{1} = poses_simu{1} * SE2.rand;

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

sploth = subplot(1, 3, 3);
% estimator
for iteration = 1:MAX_ITER
    r = zeros(NUM_MEAS, 1);
    J = zeros(NUM_MEAS, NUM_STATES);
    row = 1;
    
    delta_pose = poses_simu{1}.inv * poses{1};
    r(row:(row + 2), 1) = se2vec(delta_pose.log);
    
    cols = 1:3;
    J(row:(row+2), cols) = eye(3, 3); 
    
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































