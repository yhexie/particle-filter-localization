% Display observation model pdf
clear
clc
close all

laser_max_range = 81;
std_dev_hit = 2;
lambda_short = 0.03; % Used to calculate the chance of hitting random people or unmapped obstacles

% zParams = [0.7 0.2 0.0075 0.1]; % Weights for beam model [zHit zShort zMax zNoise]
zParams = [0.3 0.15 0.0075 0.1]; % Weights for beam model [zHit zShort zMax zNoise]

zParams = zParams / sum(zParams)
zHit = zParams(1);
zShort = zParams(2);
zMax = zParams(3);
zRand = zParams(4);


z_expected = 50;

zt = 0:0.001:laser_max_range;

pHit = calcProbHit(zt, z_expected, laser_max_range, std_dev_hit);
pShort = calcProbShort(zt, z_expected, laser_max_range, lambda_short);
pMax = calcProbMax(zt, z_expected, laser_max_range);
pRand = calcProbRand(zt, z_expected, laser_max_range);

p = zHit*pHit + zShort*pShort + zMax*pMax + zRand*pRand;

p = p./sum(p);
% figure, plot(pHit)
% figure, plot(pShort)
% figure, plot(pMax)
% figure, plot(pRand)
figure, plot(p)
figure, plot(p.^(1/2))
