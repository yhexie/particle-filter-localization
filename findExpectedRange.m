function [ z_expected ] = findExpectedRange(angle_deg, position, map, laserRange_m, occupied_threshold, map_resolution)
%findExpectedRange Summary of this function goes here
%   Detailed explanation goes here



laserStart = position';
laserEnd = zeros(1,2);
% laserRange_m = 90;
% occupied_threshold = 0.7;


laserEnd(1) = laserStart(1) + cosd( angle_deg )*laserRange_m;
laserEnd(2) = laserStart(2) + sind( angle_deg )*laserRange_m;

laserStart = laserStart / map_resolution;
laserEnd = laserEnd / map_resolution;

[rayVal,~,~,rayX,rayY] = bresenham(map, [laserStart; laserEnd], 0, occupied_threshold );
z_expected = map_resolution * sqrt((laserStart(1)-rayX(end))^2 + (laserStart(2)-rayY(end))^2);


end

