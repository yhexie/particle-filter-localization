function [ probMax ] = calcProbMax(z, z_expected, z_max )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

probMax = ones(size(z));
probMax(z<z_max) = 0;

end

