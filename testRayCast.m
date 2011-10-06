
% laserStart = 800*rand(1,2);
% laserStart = [463, 577];
laserStart = [579, 764];
laserEnd = zeros(1,2);
laserRange_m = 90;%81.83;
occupied_threshold = 0.7;

robot_theta = 2*pi/2;

for r = 1:180
    
    laserEnd(1) = laserStart(1) + cosd( (r-0.5) + ( robot_theta * 180/pi - 90) )*laserRange_m;
    laserEnd(2) = laserStart(2) + sind( (r-0.5) + ( robot_theta * 180/pi - 90) )*laserRange_m;
    [rayVal,~,~,rayX,rayY] = bresenham(wean, [laserStart; laserEnd], 1, occupied_threshold );
    z_expected = sqrt((laserStart(2)-rayX(end))^2 + (laserStart(1)-rayY(end))^2)
    refresh
    pause(0.001)
end