% Main script for particle filter

% Notes:
% We are in NED (north-east-down) coordinate system
% particle_mat: 
%   3-by-n matrix, where each column is a particle in the form
%   [x y theta]' in meters and radians [0,2pi)

%% Reset
clc
close all

%% Set parameters
global numParticles occupied_threshold laser_max_range std_dev_hit lambda_short zParams

numParticles = 1000; % Number of particles
occupied_threshold = 0.89; % Cells less than this are considered occupied
laser_max_range = 100; % Maximum laser range in meters
std_dev_hit = 0.1; % Standard deviation error in a laser range measurement
lambda_short = 0.1; % Used to calculate the chance of hitting random people or unmapped obstacles
zParams = [0.7 0.2 0.07 0.03]; % Weights for beam model [zHit zShort zMax zNoise]

% odom_params:
%   4-by-1 vector of odometry error parameters
odom_params = [0.1 0.1 0.01 0.01 ]';
% odom_params = zeros(4,1);

mapPath = 'data/map/wean.dat';

%% Load data
global_map = load(mapPath);
map_resolution = 0.1;


%% Precompute ray casts?

%% Generate random starting particles in free space

% Seed the random number generate so we can get repeatable results
rng(1);

% Find all free space on the map
[freeCellsX, freeCellsY] = find(global_map > occupied_threshold);
freeCellIndices = randperm(length(freeCellsX));

% Randomly place particles in the free space
particle_mat = [    map_resolution*freeCellsX(freeCellIndices(1:numParticles))'; 
                    map_resolution*freeCellsY(freeCellIndices(1:numParticles))'; 
                    2*pi*rand(1, numParticles)];

% Display the initial particle positions
figure
imshow(global_map);
hold on;
plot(particle_mat(2,:)./map_resolution, particle_mat(1,:)./map_resolution, 'x', 'MarkerSize', 3);



%% Loop for each log reading

logLength = 20;

for k = 2:logLength
    
    % action:
    %   6x1 matrix that expresses the two pose estimates obtained by
    %   the robot’s odometry in the form [x_prev y_prev theta_prev x_cur y_cur theta_cur]’
    action = [ 0 0 pi/2 0 0.01 pi/2 ]';
    

    
    particle_mat = move_particle(action, particle_mat, odom_params);
    
    hold on;
    plot(particle_mat(2,:)./map_resolution, particle_mat(1,:)./map_resolution, 'g.', 'MarkerSize', 3);


% If observation occured
%   Generate weights
%   Update weights
%   Resample?

end