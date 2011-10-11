% Main script for particle filter

% Notes:
% particle_mat:
%   3-by-n matrix, where each column is a particle in the form
%   [x y theta]' in meters and radians [0,2pi)

%% Reset
clc
close all
clear

if matlabpool('size') ~= 0 % checking to see if my pool is already open
     matlabpool close
     matlabpool open
else
    matlabpool open
end

%% Set parameters

mapPath = 'data/map/wean.dat';
logPath = 'data/log/robotdata1.log';
global numParticles occupied_threshold laser_max_range std_dev_hit lambda_short zParams map_resolution

numParticles = 10000; % Number of particles
w = ones(numParticles,1) / numParticles; % Particle weights - begin with uniform weight
norm_w = w./sum(w);
occupied_threshold = 0.89; % Cells less than this are considered occupied
laser_max_range = 81.8300; % Maximum laser range in meters
std_dev_hit = 0.2; % Standard deviation error in a laser range measurement
lambda_short = 0.1; % Used to calculate the chance of hitting random people or unmapped obstacles
zParams = [0.7 0.2 0.0075 0.1]; % Weights for beam model [zHit zShort zMax zNoise]
% zParams = [0.6 0 0.1 0.3]
zParams = zParams / sum(zParams)

%spacing between laser hits being considered
num_interval=5;

% odom_params:
%   4-by-1 vector of odometry error parameters
% odom_params = [0.001 0.001 0.0001 0.0001 ]';
odom_params = [0.05 0.01 0.005 0.0005]';
% odom_params = zeros(4,1);



%% Load data
[global_map,map_size,auto_shift,map_dim,resolution]  = readmap(mapPath);
map_resolution = 1/resolution;

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
% global_map_thresholded(global_map_thresholded==1) = -2;
global_map_thresholded(global_map_thresholded==-1) = 1;
global_map_thresholded(global_map_thresholded>0.3) = 1;
% global_map_thresholded(global_map_thresholded==-2) = 0.85;

global_map_thresholded = ones(size(global_map_thresholded)) - global_map_thresholded; % Invert the map
% figure, imshow(global_map_thresholded);
likelihood_field = imfilter(global_map_thresholded, h);
% likelihood_field(likelihood_field>0.4) = 1.0;
% figure(10), imshow(likelihood_field);


%% Precompute ray casts?


%% Generate random starting particles in free space

% Seed the random number generate so we can get repeatable results
%rng(1);

[ particle_mat ] = generateRandomParticles( numParticles, global_map, map_resolution, occupied_threshold );


k = 1;
best_particle = 1;
robo_mask = generate_robo_mask(.2,.3);
figure(1)
%visualize_pf(global_map, [.1 .1], particle_mat', w, z_range(1,1:180), robo_mask, particle_mat(:,best_particle)', k);

%% Loop for each log reading

logLength = length(p_robot);
count = 1;

laser_data = zeros(1,180);
for k = 200:logLength
    
    % action:
    %   6x1 matrix that expresses the two pose estimates obtained by
    %   the robot’s odometry in the form [x_prev y_prev theta_prev x_cur y_cur theta_cur]’
    action = [p_robot(k-1,1:3) p_robot(k,1:3)]';
    
    recursion_count = 0;
    particle_mat = move_particle(action, particle_mat, odom_params, global_map, recursion_count);
    
    if observation_index(k) > 1
        laser_data = z_range(observation_index(k),1:180);
    else
       % laser_data = zeros(1,180);
    end
    
    
    
    if observation_index(k) > 1
      %% If observation occured  
        % Verify time stamps
        assert ( z_range(observation_index(k), end) == p_robot(k,end) );
        
         % Pull out the sensor reading for this time stamp
        zt = z_range(observation_index(k), 1:180);

%         figure(2)
%         plot(norm_w, '.');
%         [~, best_particle] = max(w);
%         ylim([ 0 1.25*max(norm_w)])
%         
%         figure(1)
%         clf
%         visualize_pf(global_map, [.1 .1], particle_mat', w, zt, robo_mask, particle_mat(:,best_particle)', k);
        
        tic
        lostFlag = 1;
        if all(std(particle_mat') < [3 3 pi/4])
            lostFlag = 0;
        else
            lostFlag = 1;
            disp('Lost...');
        end
        
        %if lostFlag
            parfor i = 1:numParticles
                w(i) = w(i)*beam_range_finder_model( zt, particle_mat(:,i), global_map, laser_max_range, std_dev_hit, lambda_short, zParams, occupied_threshold, map_resolution,num_interval);
            end
            
        %else
         %   for i = 1:numParticles
         %       w(i) = w(i)*likelihood_field_range_finder_model( zt, particle_mat(:,i), likelihood_field, laser_max_range, std_dev_hit, lambda_short, zParams, occupied_threshold, map_resolution );
         %   end
        %end
        
        toc
        count = count + 1;
        
        
        % Normalize weights
        if (sum(w) == 0)
            w(:) = 1/numParticles;
        else
            w = w./sum(w);
            w = w.^(1/50);
            w = w./sum(w);
        end
        
        
        figure(2)
        plot(w, '.');
        [~, best_particle] = max(w);
        ylim([ 0 1.25*max(w)])
        
        figure(1)
        clf
        visualize_pf(global_map, [.1 .1], particle_mat', w, zt, robo_mask, particle_mat(:,best_particle)', k,num_interval);
        refresh
        pause(0.1)
        
        % Check if we should reduce the number of particles?
        
        if count == 3
            numParticles = round(numParticles/10);
        end
        % Resample
%         if ((ESS(w) < 0.1*numParticles))
        if ((ESS(w) < 0.7))
            %         if (mod(observation_index(k),5) == 0)
            disp('Resampling...');
            new_particle_mat = stochastic_resample(w, particle_mat', numParticles);
            particle_mat = new_particle_mat';
            w = ones(numParticles, 1)/numParticles;
        end
        
        
    end % End observation update
    
    
    
    if observation_index(k) == 0
%         figure(2)
%         plot(w, '.');
%         [~, best_particle] = max(w);
%         ylim([ 0 1.25*max(w)])
%         
%         figure(1)
%         clf
%         visualize_pf(global_map, [.1 .1], particle_mat', w, laser_data, robo_mask, particle_mat(:,best_particle)', k,num_interval);
    end
    count
    k
    
end
