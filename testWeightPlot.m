[z,laser_hit] =  findExpectedRange_(0, [40,40], global_map, 80, 0.3, 0.1,5);
angle_deg = 0
angles = -pi/2:pi/180:pi/2-pi/180;
angles = angles + deg2rad(angle_deg);
s = max(size(angles));
angles = angles(1:5:s)
r_vec = [cos(angles'),sin(angles'),zeros(size(angles'))];
r_vec = r_vec.*repmat(z,1,3);
imshow(global_map)
hold on
plot(r_vec(:,2)/0.1+400,r_vec(:,1)/0.1 + 400,'r')
plot(laser_hit(:,2),laser_hit(:,1),'b')
particle_mat = repmat([40;40;0],1,1000);


particle_mat(3,:) = -pi:pi/500:pi-pi/500;
zt = z;
numParticles = 1000
for i = 1:numParticles
                %w(i) = w(i)*beam_range_finder_model( zt, particle_mat(:,i), global_map, laser_max_range, std_dev_hit, lambda_short, zParams, occupied_threshold, map_resolution,num_interval);
                [lw,~] = beam_range_finder_model( zt,particle_mat(:,i), global_map, laser_max_range, std_dev_hit, lambda_short, zParams, occupied_threshold, 0.3,5);
                w(i) = lw*w(i);
                i
end
figure, plot(w)
norm_w = w./sum(w);
norm_w = norm_w.^(1/10);
norm_w = norm_w./sum(norm_w);
figure, plot(norm_w)