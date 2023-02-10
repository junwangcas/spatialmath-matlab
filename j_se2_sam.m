clear;
DoF = 3;
Dim = 2;

NUM_POSES = 3;
NUM_LMKS = 5;
NUM_FACTORS = 9;
NUM_STATES = NUM_POSES * DoF + NUM_LMKS * Dim;
NUM_MEAS = NUM_POSES * DoF + NUM_FACTORS * Dim;
MAX_ITER = 20;

% Define robot pose elements
X_simu = SE2;
Xi = SE2;
Xj = SE2;
poses = []
poses_simu = []

% Define a control vector and its noise and covariance in the tangent of SE2
u = [0, 0, 0]';          % control signal, generic
u_nom = [0.1, 0.0, 0.05]';      % nominal control signal
u_sigmas = [0.01, 0.01, 0.01]';   % control noise std specification
u_noise = ;    % control noise
Q;          % Covariance
W;          % sqrt Info
controls = [];   % robot controls

w
%+100.000   +0.000   +0.000
%  +0.000 +100.000   +0.000
%  +0.000   +0.000 +100.000


