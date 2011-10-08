function mask = generate_robo_mask(rs,rb)
%generates robot shape to be displayed
for i=1:20
    mask(i,1) = rs*cos((i-1)*pi/20 - pi/2);
    mask(i,2) = rs*sin((i-1)*pi/20 - pi/2);
end
n = size(mask,1);
for i=n+1:n+21
    mask(i,1) = rb*cos((i-n-1)*pi/20 + pi/2);
    mask(i,2) = rb*sin((i-n-1)*pi/20 + pi/2);
end
end