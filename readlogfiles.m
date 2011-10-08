function [p_robot,p_robot_laser,p_laser,range] = readlogfiles(fname)
% p_robot is odometry [x,y,theta,time]
% p_robot_laser is the position of the robot at the time of laser reading [x,y,theta,time]
% p_laser is the position of the laser at the time of laser reading [x,y,theta,time]
% range is the range readings [ranges time]
fid = fopen(fname);
p_robot=[];
p_robot_laser=[];
p_laser=[];
range=[];
tline = fgetl(fid);
while ischar(tline)
    if(tline(1)=='O')
        o = str2num(tline(2:end));
        p_robot = [p_robot;o];
    elseif(tline(1)=='L')
        o = str2num(tline(2:end));
        p_robot = [p_robot;o(1:3),o(end)];
        p_robot_laser = [p_robot_laser;o(1:3),o(end)];
        p_laser= [p_laser;o(4:6),o(end)];
        range = [range;o(7:end)];
    end
    tline = fgetl(fid);
end

% Convert to SI units
p_robot(:,1:2) = p_robot(:,1:2) ./ 100; % cm to meters
p_robot_laser(:,1:2) = p_robot_laser(:,1:2) ./ 100; % cm to meters
p_laser(:,1:2) = p_laser(:,1:2) ./ 100; % cm to meters
range(:,1:180) = range(:,1:180) ./ 100; % cm to meters

% Convert to NED coordinates
p_robot(:,2:3) = p_robot(:,2:3) * -1;
p_robot_laser(:,2:3) = p_robot_laser(:,2:3) * -1;
p_laser(:,2:3) = p_laser(:,2:3) * -1;

fclose(fid);