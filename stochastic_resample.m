function xn = stochastic_resample(w,x, numParticles,number_particles_wanted)
% figure
% scatter3(x(:,1),x(:,2),w)
w = w/sum(w);
wc  = cumsum(w);
wc(end) = 1;
num_to_add = 1/numParticles;
rand_number = wc(find(wc>=rand(1),1,'first'));
rand_number = repmat(rand_number,numParticles,1);
rand_number = rand_number + (1:numParticles)'*num_to_add;
rand_number  = rem(rand_number,1);
ind = zeros(numParticles,1);
for i=1:numParticles
   [ind(i),~] =  find(wc>=rand_number(i),1,'first');
   
end

xn = x(ind,:);

wanted_index = ceil(rand(number_particles_wanted,1)*numParticles);
xn = xn(wanted_index,:);