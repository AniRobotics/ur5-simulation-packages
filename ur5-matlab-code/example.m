clc
clear

% This is an example of computing spatial and analytical Jacobian
% of a serial chain manipulator. The paprameters used here are for
% UR5 robot.

% Input: Joint angles of Left Arms
% theta = zeros(1, 6);
theta = 0.1*ones(1, 6);

% Orientation of the frames
R1 = quat2rotm([0.000, 0.000, 1.000, -0.000]);
R2 = quat2rotm([-0.000, -0.707, -0.000, 0.707]);
R3 = quat2rotm([-0.000, -0.707, -0.000, 0.707]);
R4 = quat2rotm([0.000, 1.000, 0.000, -0.000]);
R5 = quat2rotm([0.000, 1.000, 0.000, -0.000]);
R6 = quat2rotm([0.000, 1.000, 0.000, -0.000]);

% Axis of rotations
w1 = R1(:, 3); w2 = R2(:, 2); w3 = R3(:, 2);
w4 = R4(:, 2); w5 = R5(:, 3); w6 = R6(:, 2);
wr = [w1, w2, w3, w4, w5, w6];

% Frame origins
q1 = [0.000; 0.000; 0.089];   q2 = [0.000; -0.136; 0.089];
q3 = [-0.425; -0.016; 0.089]; q4 = [-0.817; -0.016; 0.089];
q5 = [-0.817; -0.109; 0.089]; q6 = [-0.817; -0.109; -0.005];
qr = [q1, q2, q3, q4, q5, q6];

% Define g_st0 (Tool frame)
g_st0 = [quat2rotm([0.707, 0.707, 0.000, -0.000]), [-0.817; -0.191; -0.005]; 
         zeros(1, 3), 1];

% Compute jacobian matrices
[spatial_jac, analytical_jac] = spatial_jacobian_mat(qr, wr, theta, g_st0);

fprintf("Joint angle vector: \n");
disp(theta);
fprintf("Spatial Jacobian: \n");
disp(spatial_jac);