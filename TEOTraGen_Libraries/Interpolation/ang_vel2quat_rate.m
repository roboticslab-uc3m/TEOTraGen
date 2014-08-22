function [ q_rate ] = ang_vel2quat_rate( q, w )
%ANG_VEL2QUAT_RATE Summary of this function goes here
%   Detailed explanation goes here

% Author: Domingo Esteban

W = [-q(1)  q(0) -q(3)  q(2);
     -q(2)  q(3)  q(0) -q(1);
     -q(3) -q(2) -q(1)  q(0)];

q_rate = 1/2*W'*w;

end