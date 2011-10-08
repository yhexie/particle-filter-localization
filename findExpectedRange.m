function [ z_expected ] = findExpectedRange(angle_deg, position, map, laserRange_m, occupied_threshold, map_resolution)
%#codegen
%findExpectedRange Summary of this function goes here
%   Detailed explanation goes here

drawFlag = 0;

laserStart = position';
laserEnd = zeros(1,2);
% laserRange_m = 90;
% occupied_threshold = 0.7;


laserEnd(1) = laserStart(1) + cosd( angle_deg )*laserRange_m;
laserEnd(2) = laserStart(2) + sind( angle_deg )*laserRange_m;

laserStart_map = laserStart / map_resolution;
laserEnd_map = laserEnd / map_resolution;

hold on;
% plot([laserStart_map(2) laserEnd_map(2)], [laserStart_map(1) laserEnd_map(1)], 'c'); 
% refresh
% pause(0.1);
start_endPoints = [laserStart_map; laserEnd_map];

[rayVal,~,~,rayX,rayY] = bresenham(map, start_endPoints, drawFlag, occupied_threshold );

if drawFlag
    refresh;
    pause(0.1);
end

rayEnd_map = [rayX(end) rayY(end)];
z_expected = norm(laserStart_map - rayEnd_map) * map_resolution;

z_expected(z_expected>laserRange_m) = laserRange_m;


end

