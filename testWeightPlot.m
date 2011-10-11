z =  findExpectedRange_(0, [40,40], global_map, 80, 0.3, 0.1)
angle_deg = 0
angles = -pi/2:pi/180:pi/2-pi/180;
angles = angles + deg2rad(angle_deg);
r_vec = [cos(angles'),sin(angles'),zeros(size(angles'))];
r_vec = r_vec.*repmat(z,1,3);
imshow(global_map)
hold on
plot(r_vec(:,2)/0.1+400,r_vec(:,1)/0.1 + 400,'r')

particle_mat = repmat([40;40;0],1,1000);


particle_mat(3,:) = -pi:pi/500:pi-pi/500;
zt = z;
numParticles = 1000
parfor i = 1:numParticles
w(i) = w(i)*beam_range_finder_model( zt, particle_mat(:,i), global_map, laser_max_range, std_dev_hit, lambda_short, zParams,0.3, map_resolution );
end
figure, plot(w)
norm_w = w./sum(w);
norm_w = norm_w.^(1/1000);
figure, plot(norm_w)