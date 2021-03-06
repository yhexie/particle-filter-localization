function [ ess ] = ESS( w )
w = w/sum(w);
M = size(w,1);
cv = sum( ( w.*M - 1 ).^2 ) / M;
ess = 1 / ( 1 + cv );