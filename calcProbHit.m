function [ probHit ] = calcProbHit(z, z_expected, z_max, std_dev_hit)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

assert(z_max >= z_expected);
assert(z_max >= z);

if (z >= 0 && z <= z_max)
    
    probHit = normpdf(z, z_expected, std_dev_hit);
    
    normalizor =  (sum(normpdf(0:0.01:z_max, z_expected, std_dev_hit)) * 0.01);
    probHit = probHit/normalizor;
else
    probHit = 0;
end

end

