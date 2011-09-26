function visualize_pf(map,scale,x,w,observation,mask,selected_particle)

im = mat2gray(map);
%%particle display
xx = x;
x(:,1) = x(:,1)/scale(1);
x(:,2) = x(:,2)/scale(2);

x(x(:,1)<0,:) = [];
x(x(:,2)<0,:) = [];
x(x(:,1)>size(map,1),:) = [];
x(x(:,2)>size(map,2),:) = [];

num_bins = uint32(max(max(x) - min(x)));

n = max(hist(double(x(:,1)),double(num_bins)));

x = uint32(x);

im(:,:,1) = 0;
for i=1:size(x,1)
    
    im(x(i,1),x(i,2),1)  = im(x(i,1),x(i,2),1) + (size(x,1)/n)*w(i);
    im(x(i,1),x(i,2),2)  = 0;
    im(x(i,1),x(i,2),3)  = 0;

end
im(:,:,1) = im(:,:,1)/max(max(im(:,:,1)));

%% robot display
q = [cos(selected_particle(3)/2) 0 0 -sin(selected_particle(3)/2)];

mask = quatrotate(q,[mask,zeros(size(mask,1),1)]);
mask(:,1) = mask(:,1)/scale(1) + selected_particle(1)/scale(1);
mask(:,2) = mask(:,2)/scale(2) + selected_particle(2)/scale(2);
mask = uint32(round(mask));
for j=1:size(mask,1)
   im(mask(j,1),mask(j,2),1) = 0; 
   im(mask(j,1),mask(j,2),2) = 1; 
   im(mask(j,1),mask(j,2),3) = 0; 
end
%% observation display
angles = -pi/2:pi/180:pi/2;
r_vec = [cos(angles),sin(angles),0];
r_vec = quatrotate(q,r_vec);
r_vec = observation.*r_vec;
r_vec(:,1) = r_vec(:,1)/scale(1);
r_vec(:,2) = r_vec(:,2)/scale(2);
r_vec  = uint32(round(r_vec));

for i = 1:size(r_vec,1)
    m = mean(im(r_vec(i,1),r_vec(i,2),1:3));
   im(r_vec(i,1),r_vec(i,2),1) = m; 
   im(r_vec(i,1),r_vec(i,2),2) = m; 
   im(r_vec(i,1),r_vec(i,2),3) = m; 
end
im(im<0) = 0;
im(im>1) = 1;
imshow(im)
% use plot to display lines
end