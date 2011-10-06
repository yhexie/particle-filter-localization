function [ probRand ] = calcProbRand(z, z_expected, z_max)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

if (z >= 0 && z <= z_max)
    
    probRand = 1 / z_max;
else
    probShort = 0;
end

end

