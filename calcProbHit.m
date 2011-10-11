function [ probHit ] = calcProbHit(z, z_expected, z_max, std_dev_hit)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

if (z_expected > z_max )
    keyboard
end

probHit = normpdf(z, z_expected, std_dev_hit);
%     normalizor =  (sum(normpdf(0:0.01:z_max, z_expected, std_dev_hit)) * 0.01);
normalizor = 1;
probHit = probHit/normalizor;

probHit(z < 0) = 0;
probHit(z > z_max) = 0;

end

