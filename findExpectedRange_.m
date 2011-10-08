function [ z_expected ] = findExpectedRange_(angle_deg, position, map, laserRange_m, occupied_threshold, map_resolution)
tic
r = 0:laserRange_m/map_resolution;
r=r';
r = repmat(r,1,3);
r = reshape(r',1,[]);
r = repmat(r,180,1);
angles = -pi/2:pi/180:pi/2-pi/180;
angles = angles + deg2rad(angle_deg);
r_vec = [cos(angles'),sin(angles'),zeros(size(angles'))];
r_vec = repmat(r_vec,1,size(r,2)/3);

r_vec = r.*r_vec;
r_vec = floor(r_vec);

z_expected = zeros(size(r_vec,1),1);

for i=1:size(r_vec,1)
    ray = r_vec(i,:);
    ray = reshape(ray,3,[]);
    ray = ray';

    ray(:,1) = ray(:,1) + position(1)/map_resolution;
    ray(:,2) = ray(:,2) + position(2)/map_resolution;
    rr = 0:laserRange_m/map_resolution;

    ray(:,3) = rr;

    ray(ray(:,1)>800,:) = [];
    ray(ray(:,2)>800,:) = [];

    ray(ray(:,1)<1,:) = [];
    ray(ray(:,2)<1,:) = [];

    ray(:,1:2) = floor(ray(:,1:2));

     H = sub2ind(size(map),ray(:,1),ray(:,2));
     a =  find(map(H)<occupied_threshold,1,'first');
     if(isempty(a))
        a =laserRange_m;
     end
     z_expected(i) = ray(a,3);

end
z_expected  = z_expected*map_resolution;
toc
