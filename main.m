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
logPath = 'data/log/robotdata2.log';
global numParticles laser_max_range std_dev_hit lambda_short zParams map_resolution

numParticles = 10000; % Number of particles
w = ones(numParticles,1) / numParticles; % Particle weights - begin with uniform weight
free_threshold = 0.89; % Cells less than this are considered occupied
occupied_threshold = 0.1;

laser_max_range = 81.8300; % Maxim`um laser range in meters
std_dev_hit = 0.2; % Standard deviation error in a laser range measurement
% std_dev_hit = 2;
% lambda_short = 0.1; % Used to calculate the chance of hitting random people or unmapped obstacles
lambda_short = 0.1;
zParams = [0.7 0.2 0.1 0.1]; % Weights for beam model [zHit zShort zMax zNoise]
% zParams = [0.6 0 0.1 0.3]
% zParams = [0.3 0.15 0.0075 0.1];
zParams = zParams / sum(zParams)

%spacing between laser hits being considered
num_interval=1;
laser_hit_p = zeros([max(size(1:num_interval:180)),2,numParticles]);

% odom_params:
%   4-by-1 vector of odometry error parameters
odom_params = [0.001 0.001 0.0001 0.0001 ]';
% odom_params = [0.05 0.01 0.005 0.0005]';



%% Load data
[global_map,map_size,auto_shift,map_dim,resolution]  = readmap(mapPath);
map_resolution = 1/resolution;

global_map(global_map == -1) = 0.5;

display_map = global_map;
display_map(display_map>occupied_threshold) = 1;
% figure, imshow(display_map)

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

%% Generate random starting particles in free space

% Seed the random number generate so we can get repeatable results
% rng(2);

[ particle_mat ] = generateRandomParticles( numParticles, global_map, map_resolution, free_threshold );
%[ particle_mat_ ] = generateFocussedRandomParticles( numParticles/10, global_map, map_resolution, free_threshold,[40,41.3,0] );
%particle_mat = [particle_mat(:,1:numParticles-numParticles/10),particle_mat_];



k = 1;
best_particle = 1;
robo_mask = generate_robo_mask(.2,.3);
figure(1)
%visualize_pf(global_map, [.1 .1], particle_mat', w, z_range(1,1:180), robo_mask, particle_mat(:,best_particle)', k);

%% Loop for each log reading

logLength = length(p_robot);
count = 1;

v = diff(p_robot);
dx = v(:,1)./v(:,4);
dy = v(:,2)./v(:,4);
dtheta = abs(v(:,3)./v(:,4));
velocity = sqrt(dx.^2 + dy.^2);
% figure, plot(velocity, '.')
% figure, plot(dtheta, '.')


laser_data = zeros(1,180);
for k = 2:logLength
    
    % action:
    %   6x1 matrix that expresses the two pose estimates obtained by
    %   the robot’s odometry in the form [x_prev y_prev theta_prev x_cur y_cur theta_cur]’
    action = [p_robot(k-1,1:3) p_robot(k,1:3)]';
    
    recursion_count = 0;
    [particle_mat, w] = move_particle(action, particle_mat, w, odom_params, global_map, free_threshold, recursion_count);
    numParticles = length(particle_mat);
    
    if observation_index(k) > 1
        laser_data = z_range(observation_index(k),1:180);
    else
        % laser_data = zeros(1,180);
    end
    
    
    hasMovement = velocity(k) > 0.1 || dtheta(k) > 0.01;
    
    if (observation_index(k) > 1 && hasMovement)
        %% If observation occured
        % Verify time stamps
        assert ( z_range(observation_index(k), end) == p_robot(k,end) );
        
        % Pull out the sensor reading for this time stamp
        zt = z_range(observation_index(k), 1:180);
        
        %         figure(2)
        %         plot(w, '.');
        %         [~, best_particle] = max(w);
        %         ylim([ 0 1.25*max(norm_w)])
        %
        %         figure(1)
        %         clf
        %         visualize_pf(global_map, [.1 .1], particle_mat', w, zt, robo_mask, particle_mat(:,best_particle)', k);
        
        tic
        
        lw = ones(numParticles,1);
%         if mod(count,6) == 0
        if round(rand(1))

            noise_val = 1/15;
            num_interval=10;
            laser_hit_p = zeros([max(size(1:num_interval:180)),2,numParticles]);
            display('BeamRangeModel')
            parfor i = 1:numParticles
                %w(i) = w(i)*beam_range_finder_model( zt, particle_mat(:,i), global_map, laser_max_range, std_dev_hit, lambda_short, zParams, occupied_threshold, map_resolution,num_interval);
                [lw(i),laser_hit_p(:,:,i)] = beam_range_finder_model( zt,particle_mat(:,i), global_map, laser_max_range, std_dev_hit, lambda_short, zParams, occupied_threshold, map_resolution,num_interval);
                
            end
            
        else
            display('likelyhood field')
            noise_val = 1/2;
            num_interval=1;
            for i = 1:numParticles
                lw(i)=likelihood_field_range_finder_model( zt, particle_mat(:,i), likelihood_field, laser_max_range, std_dev_hit, lambda_short, zParams, occupied_threshold, map_resolution );
            end
        end
        
        toc
        count = count + 1;
        
        
        % Normalize weights
        if (sum(w) == 0)
            w(:) = 1/numParticles;
        else
            
            w = w./sum(w);
            lw = lw./sum(lw);
%             reduced_lw = lw.^(1/20);
            reduced_lw = lw.^(noise_val);
            reduced_lw = reduced_lw./sum(reduced_lw);
            w = reduced_lw.*w;
        end
        
        
        figure(2)
        plot(w, '.');
        [~, best_particle] = max(w);
        ylim([ 0 1.25*max(w)])
        
        figure(1)
        clf
        visualize_pf(global_map, [.1 .1], particle_mat', w, zt, robo_mask, particle_mat(:,best_particle)', k,num_interval,laser_hit_p(:,:,best_particle));
        refresh
        pause(0.1)
        
        % Check if we should reduce the number of particles?
        
        ess_value = ESS(w)
        if ((ESS(w) < 0.05))
            if (count > 30)
                numParticles = 10000;
                disp('REDUCING PARTICLE COUNT');
            else
                if numParticles < 10000
                    numParticles = 10000;
                end
            end
            disp('Resampling...');
            new_particle_mat = stochastic_resample(w, particle_mat', numParticles,round(0.9*numParticles));
            [ rand_particle_mat ] = generateRandomParticles( round(0.1*numParticles), global_map, map_resolution, free_threshold );
            particle_mat = [new_particle_mat' rand_particle_mat];
            numParticles = length(particle_mat);
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
    %     count
    %     k
    
end
