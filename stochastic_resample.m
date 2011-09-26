function xn = stochastic_resample(w,x)
w = w/sum(w);
wc  = cumsum(w);
num_to_add = 1/size(wc,1);
rand_number = wc(find(wc>rand(1),1,'first'));
rand_number = repmat(rand_number,size(w,1),1);
rand_number = rand_number + (1:size(w,1))'*num_to_add;
rand_number  = rem(rand_number,1);
ind = zeros(size(w,1),1);
parfor i=1:size(wc,1)
   [ind(i),~] =  find(wc>rand_number(i),1,'first');
   
end

xn = x(ind,:);