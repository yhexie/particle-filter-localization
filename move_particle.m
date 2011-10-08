function [ new_particle_mat ] = move_particle( action, particle_mat, odom_alpha )
%move_particle Moves all the particles according to the measured action and
%               motion model

% action:   
%   6x1 matrix that expresses the two pose estimates obtained by 
%   the robot’s odometry in the form [x_prev y_prev theta_prev x_cur y_cur theta_cur]’

% particle_mat: 
%   3-by-n matrix which each row represents a single particle and is in the form [x y theta]

% odom_alpha:
%   4-by-1 vector of odometry error parameters
a = odom_alpha;

% Algorithm taken from Probabilistic Robotics, page 136 Table 5.6
odom_x_prev      = action(1);
odom_y_prev      = action(2);
odom_theta_prev  = action(3);
odom_x_cur       = action(4);
odom_y_cur       = action(5);
odom_theta_cur   = action(6);

delta_x = odom_x_cur - odom_x_prev;
delta_y = odom_y_cur - odom_y_prev;
delta_rot = odom_theta_cur - odom_theta_prev;

rot1 = atan2(delta_y , delta_x) - odom_theta_prev;
trans = sqrt((delta_x)^2 + (delta_y)^2);
rot2 = odom_theta_cur - odom_theta_prev - rot1;

numP = size(particle_mat,2);

noisy_rot1 = repmat(rot1, numP, 1)      - randn(numP,1) * sqrt( a(1)*rot1^2 + a(2)*trans^2 );
noisy_trans = repmat(trans, numP, 1)    - randn(numP,1) * sqrt( a(3)*trans^2 + a(4)*rot1^2 + a(4)*rot2^2 );
noisy_rot2 = repmat(rot2, numP, 1)      - randn(numP,1) * sqrt( a(1)*rot2^2 + a(2)*trans^2 );

% figure, hist(noisy_rot1 * 180/pi)
% figure, hist(noisy_trans)
% figure, hist(noisy_rot2 * 180/pi)

x = particle_mat(1,:);
y = particle_mat(2,:);
theta = particle_mat(3,:);

new_x = x + noisy_trans' .* cos(theta + noisy_rot1');
new_y = y + noisy_trans' .* sin(theta + noisy_rot1');
new_theta = theta + noisy_rot1' + noisy_rot2';

delta_theta = new_theta - theta;

new_particle_mat = [ new_x; new_y; new_theta];
end

