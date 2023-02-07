%% equation 25 -- 27, so2的adjoint matrix就是它自己
clear
theta1 = pi/2
R1 = SO2(theta1)
theta_skew = R1.log()

theta2 = pi/10
R2 = SO2(theta2)
R3 = R1*R2
theta_skew3 = R3.log()

R4 = R2*R1
theta_skew4 = R4.log()

%% 
