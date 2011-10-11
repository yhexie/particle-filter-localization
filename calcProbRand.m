function [ probRand ] = calcProbRand(z, z_expected, z_max)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


probRand = ones(size(z)) / z_max;
probRand(z < 0) = 0;
probRand(z >= z_max) = 0;


end

