numInterval = 6;
[z,laser_hit] =  findExpectedRange_(0, [40.25,40.0], global_map,laser_max_range, occupied_threshold, 0.1,1);

imshow(global_map)
hold on
plot(laser_hit(:,2,1),laser_hit(:,1,1),'.r')
numParticles = 1000;
 
 
particle_mat = repmat([30;40;0],1,numParticles);
 xx = 0:20/size(particle_mat,2):(20-20/size(particle_mat,2));
particle_mat(1,:) = particle_mat(1,:) + xx;

%  particle_mat(3,:) = -pi:pi/500:pi-pi/500;
zt = z;

w = ones(numParticles,1);


laser_max_range = 81.8300; % Maxim`um laser range in meters
std_dev_hit = 0.2; % Standard deviation error in a laser range measurement
% std_dev_hit = 2;
% lambda_short = 0.1; % Used to calculate the chance of hitting random people or unmapped obstacles
lambda_short = 0.01;
zParams = [0.7 0.2 0.1 0.1]; % Weights for beam model [zHit zShort zMax zNoise]
% zParams = [0.6 0 0.1 0.3]
% zParams = [0.3 0.15 0.0075 0.1];
zParams = zParams / sum(zParams)


laser_hit = [];
zp = [];
for i = 1:numParticles
                %w(i) = w(i)*beam_range_finder_model( zt, particle_mat(:,i), global_map, laser_max_range, std_dev_hit, lambda_short, zParams, occupied_threshold, map_resolution,num_interval);
                [lw,laser_hit(:,:,i)] = beam_range_finder_model( zt,particle_mat(:,i), global_map, laser_max_range, std_dev_hit, lambda_short, zParams, occupied_threshold,map_resolution,numInterval);
                w(i) = lw*w(i);
                
               
                 [zw,~] = findExpectedRange_(rad2deg(particle_mat(3,i)), particle_mat(1:2,i)'+[0.25,0.0], global_map, laser_max_range, occupied_threshold,map_resolution,numInterval);
                 zp = [zp;zw'];
                 
                 
end

% for i = 1:numParticles
%                 w(i)=likelihood_field_range_finder_model( zt', particle_mat(:,i), likelihood_field, laser_max_range, std_dev_hit, lambda_short, zParams, occupied_threshold, map_resolution );
% end
[~,max_ind]=max(w);
% plot(laser_hit(:,2,1),laser_hit(:,1,1),'.b')
% plot(laser_hit(:,2,max_ind),laser_hit(:,1,max_ind),'.g')

figure, plot(w)
norm_w = w./sum(w);
norm_w = norm_w.^(1/10);
norm_w = norm_w./sum(norm_w);
figure, plot(norm_w)

% figure
% plot(z(1:numInterval:size(z,1)))
% hold on
% plot(zp(1,:),'r')
% plot(zp(max_ind,:),'g')