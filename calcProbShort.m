function [ probShort ] = calcProbShort(z, z_expected, z_max, lambda_short)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

if (z >= 0 && z <= z_expected)
    
    normalizor = 1/(1 - exp(-lambda_short*z_expected));
    probShort = lambda_short*exp(-lambda_short*z)*normalizor;
else
    probShort = 0;
end

end

