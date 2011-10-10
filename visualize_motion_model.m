% Visualize motion model
clc
clear
close all

% action:   
%   6x1 matrix that expresses the two pose estimates obtained by 
%   the robot’s odometry in the form [x_prev y_prev theta_prev x_cur y_cur theta_cur]’
action = [0 0 0*pi/180 1 1 45*pi/180]';

% particle_mat: 
%   3-by-n matrix which each row represents a single particle and is in the form [x y theta]
single_particle = [40 40 0]';
numParticles = 10000;
particle_mat = repmat(single_particle, 1, numParticles);

% odom_params:
%   4-by-1 vector of odometry error parameters
%   (1) Rotation noise in rotation
%   (2) Translation noise in rotation
%   (3) Translation noise in Translation
%   (4) Rotation noise in Translation
% odom_params = [0.001 0.001 0.0001 0.0001 ]';
odom_params = [0.05 0.01 0.005 0.0005]';


global_map = ones(800,800);
recursion_count = 0;



[ new_particle_mat ] = move_particle( action, particle_mat, odom_params, global_map, recursion_count );

true_position = single_particle + action(4:6) - action(1:3);

figure

plot(new_particle_mat(1,:), new_particle_mat(2,:), '.', 'MarkerSize', 3);
hold on;
plot(single_particle(1,:), single_particle(2,:), 'ro');
hold on;
plot(true_position(1,:), true_position(2,:), 'gx');
hold on;
axis equal