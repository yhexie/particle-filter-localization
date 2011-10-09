function [ ess ] = ESS( w )

M = size(w,1);
cv = sum( ( w.*M - 1 ).^2 ) / M;
ess = 1 / ( 1 + cv );