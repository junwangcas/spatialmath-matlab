% z 方向无关，当成一个2d的pose就好了，而且T也是固定的。
clear;
DoF = 1;

NUM_POSES = 1;
NUM_STATES = NUM_POSES * DoF;
NUM_MEAS = NUM_POSES * DoF;
MAX_ITER = 20;

% Simulator
% poses, controls
poses = cell(NUM_POSES, 1);
poses{1} = SO2(1.1);
%poses{1} = SO2(0.0);

%% visualize
% ground truth 
% initial values
subplot(1, 2, 1);
for i = 1:NUM_POSES
    hold on; poses{i}.plot;
end
xlim([-1, 4]); ylim([-2, 3]);
%axis equal
axis auto
title('2 poses - init');

lmks = [0, 1];
T = [0, 0]';
bias_axis = [1, 0];


sploth = subplot(1, 2, 2);
% estimator
for iteration = 1:MAX_ITER
    r = zeros(NUM_MEAS, 1);
    J = zeros(NUM_MEAS, NUM_STATES);
    row = 1;
    
    delta_p = lmks(1,:)' - T;
    r(1) = bias_axis * (poses{1}.inv * delta_p);
    
    ix = poses{1}.inv;
    d_e_ix = bias_axis * (ix * skew(1) * delta_p);
    d_ix_x = -1;
    d_e_x = d_e_ix * d_ix_x;
    
    J(1) = d_e_x; 
    
    dx = - inv(J' * J) * J' * r
    % update
    %test = vec2so2([dx()]);
    %test2 = SO2.exp(test);
    %poses{1} = poses{1} * test2;
    poses{1} = poses{1} * SO2.exp(vec2so2([dx()]));
    
    % plot
    %clf;
    for i = 1:NUM_POSES
        hold on; poses{i}.plot;
    end
    xlim([-1, 4]); ylim([-2, 3]);
    %axis equal
    axis auto
    title('2 poses - optimization');
    waitforbuttonpress;
    cla(sploth);
end































