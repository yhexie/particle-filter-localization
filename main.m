% Main script for particle filter

% Notes:
% particle_mat:
%   3-by-n matrix, where each column is a particle in the form
%   [x y theta]' in meters and radians [0,2pi)

%% Reset
clc
close all
clear

%% Set parameters

mapPath = 'data/map/wean.dat';
logPath = 'data/log/robotdata1.log';
global numParticles occupied_threshold laser_max_range std_dev_hit lambda_short zParams map_resolution

numParticles = 5000; % Number of particles
w = ones(numParticles,1) / numParticles; % Particle weights - begin with uniform weight
occupied_threshold = 0.89; % Cells less than this are considered occupied
laser_max_range = 81.8300; % Maximum laser range in meters
std_dev_hit = 0.1; % Standard deviation error in a laser range measurement
lambda_short = 0.1; % Used to calculate the chance of hitting random people or unmapped obstacles
zParams = [0.7 0.2 0.07 0.03]; % Weights for beam model [zHit zShort zMax zNoise]
zParams = zParams / norm(zParams);

% odom_params:
%   4-by-1 vector of odometry error parameters
odom_params = [0.001 0.001 0.0001 0.0001 ]';
% odom_params = zeros(4,1);



%% Load data
global_map = load(mapPath);
map_resolution = 0.1;

% p_robot is odometry [x,y,theta,time]
% p_robot_laser is the position of the robot at the time of laser reading [x,y,theta,time]
% p_laser is the position of the laser at the time of laser reading [x,y,theta,time]
% z_range is the range readings [ranges time]
[p_robot,observation_index,p_laser,z_range] = readlogfiles(logPath);

%% Compute likelihood field
scaled_sigma = std_dev_hit/map_resolution;
hsize = 30;
h = fspecial('gaussian', hsize, scaled_sigma);

global_map_thresholded = global_map ;
global_map_thresholded(global_map_thresholded==-1) = 1;
global_map_thresholded(global_map_thresholded>0.2) = 1;
global_map_thresholded(global_map_thresholded==-1) = 1;
global_map_thresholded = ones(size(global_map_thresholded)) - global_map_thresholded;
% figure, imshow(global_map_thresholded);
likelihood_field = imfilter(global_map_thresholded, h);
% figure, imshow(likelihood_field);
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
% figure
% imshow(global_map);
% hold on;
% plot(particle_mat(2,:)./map_resolution, particle_mat(1,:)./map_resolution, 'rx', 'MarkerSize', 3);

k = 1;
best_particle = 1;
robo_mask = generate_robo_mask(.25,.40);
figure(1)
visualize_pf(global_map, [.1 .1], particle_mat', w, z_range(1,1:180), robo_mask, particle_mat(:,best_particle)', k);

%% Loop for each log reading

logLength = length(p_robot);

for k = 2:logLength
    
    % action:
    %   6x1 matrix that expresses the two pose estimates obtained by
    %   the robot’s odometry in the form [x_prev y_prev theta_prev x_cur y_cur theta_cur]’
    action = [p_robot(k-1,1:3) p_robot(k,1:3)]';
    
    
    
    particle_mat = move_particle(action, particle_mat, odom_params);
    
    if observation_index(k) > 1
        laser_data = z_range(observation_index(k),1:180);
    else
        laser_data = zeros(1,180);
    end
    figure(1)
    clf
    visualize_pf(global_map, [.1 .1], particle_mat', w, laser_data, robo_mask, particle_mat(:,best_particle)', k);   
    
%         hold on;
%         plot(particle_mat(2,:)./map_resolution, particle_mat(1,:)./map_resolution, 'g.', 'MarkerSize', 3);
%         refresh
%         pause(0.01)
    
    % If observation occured
    if observation_index(k) > 1
        
        % Verify time stamps
        assert ( z_range(observation_index(k), end) == p_robot(k,end) );
        
        zt = z_range(observation_index(k), 1:180);
        tic
        for i = 1:numParticles
            
            
            %   Generate and update weights
%             w(i) = w(i)*beam_range_finder_model( zt, particle_mat(:,i), global_map );
            w(i) = w(i)*likelihood_field_range_finder_model( zt, particle_mat(:,i), likelihood_field );
        end
        toc
%         Normalize weights
                w = w./norm(w);
                figure(2)
                plot(w, '.');
                [~, best_particle] = max(w);
                
                if (mod(k,20) == 0)
                    new_particle_mat = stochastic_resample(w, particle_mat');
                    w(:) = 1/numParticles;
                end
        
        
        
    end
    
end
