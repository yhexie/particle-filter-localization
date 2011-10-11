function [ probShort ] = calcProbShort(z, z_expected, z_max, lambda_short)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


normalizor = 1/(1 - exp(-lambda_short*z_expected));
probShort = lambda_short*exp(-lambda_short*z)*normalizor;

probShort(z < 0) = 0;
probShort(z > z_expected) = 0;

end

