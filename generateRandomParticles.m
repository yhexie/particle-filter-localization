function [ particle_mat ] = generateRandomParticles( numParticles, global_map, map_resolution, free_threshold )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Find all free space on the map
[freeCellsX, freeCellsY] = find(global_map > free_threshold);
freeCellIndices = randperm(length(freeCellsX));

% Randomly place particles in the free space
particle_mat = [    map_resolution*freeCellsX(freeCellIndices(1:numParticles))';
    map_resolution*freeCellsY(freeCellIndices(1:numParticles))';
    2*pi*rand(1, numParticles)];


end

