function [ likelihood ] = likelihood_field_range_finder_model( zt, xt, lh_field )
%beam_range_finder_model Summary of this function goes here
%   Detailed explanation goes here

% zt:
%   A 180-by-1 vector of laser range values reported from the sensor

% xt:
%   A 3-by-1 vector for the robot's position and orientation

% lh_field:
%   The likelihood field for the map

global laser_max_range std_dev_hit lambda_short zParams occupied_threshold map_resolution


zHit = zParams(1);
zShort = zParams(2);
zMax = zParams(3);
zRand = zParams(4);


numLaserScans = length(zt);

robot_angle_rad = xt(3);
robot_angle_deg = robot_angle_rad * 180/pi;
robot_position_m = xt(1:2);
laser_position_m = robot_position_m + [ 0.25*cosd(robot_angle_deg); 
                                        0.25*sind(robot_angle_deg)];
               
interval = 10;                                    
laser_angles_deg = 0.5:interval:179.5;   
laser_positions_m = repmat(laser_position_m, 1, length(laser_angles_deg));

laser_end_position_m = laser_positions_m + [zt(1:interval:end) .* cosd(laser_angles_deg + robot_angle_deg - 90);
                                            zt(1:interval:end) .* sind(laser_angles_deg + robot_angle_deg - 90)];


scale = 0.1;
x_index = round(laser_end_position_m(1,:)'./scale);
y_index = round(laser_end_position_m(2,:)'./scale);

mask = x_index>size(lh_field,1) | x_index<1 | y_index>size(lh_field,2) | y_index<1;
% if any(mask)
%     keyboard
% end
valid_x_index = x_index(~mask);
valid_y_index = y_index(~mask);
valid_zt = zt(~mask);



% if (any(x_index>size(lh_field,1)) || any(x_index<1) || any(y_index>size(lh_field,2)) || any(y_index<1))
%     likelihood = (zRand/zMax)^length(laser_angles_deg);
%     return;
% end

end_point_index = sub2ind(size(lh_field), valid_x_index, valid_y_index);
% false_field = zeros(size(lh_field));
% false_field(end_point_index) = 1;
% figure; imagesc(false_field);


q = zHit*lh_field(end_point_index) + 1; %+ repmat(zRand/zMax, length(end_point_index), 1);
q(valid_zt >= laser_max_range) = 1;

% if (length(laser_angles_deg)-length(q) ~=0)
%     keyboard
% end
 if (length(laser_angles_deg)-length(q) ==0)
    likelihood = prod(q); 
 else
     likelihood = prod(q) * 0.9^(length(laser_angles_deg)-length(q));%(zRand/zMax)^(length(laser_angles_deg)-length(q));
 end

% figure, imshow(lh_field)
% hold on;
% plot(laser_end_position_m(2,:)*10, laser_end_position_m(1,:)*10, 'r.');
% figure, plot(q)
% keyboard
end

