global laser_max_range std_dev_hit lambda_short zParams occupied_threshold

zParams = [0.7 0.2 0.07 0.03]; % [zHit zShort zMax zRand]

laser_max_range = 100;
std_dev_hit = 0.1;
lambda_short = 0.1;

occupied_threshold = 0.7;

xt = [ 582 657 0]'
zt = 1
[ q ] = beam_range_finder_model( zt, xt, wean )