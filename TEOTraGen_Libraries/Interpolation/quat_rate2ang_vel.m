function [ w ] = quat_rate2ang_vel( q, q_rate )
%QUAT_RATE2ANG_VEL Summary of this function goes here
%   Detailed explanation goes here

% Author: Domingo Esteban


nelements = size(q,2);
w = zeros(3,nelements);

for i=1:nelements,

  W = [-q(2,i)  q(1,i) -q(4,i)  q(3,i);
       -q(3,i)  q(4,i)  q(1,i) -q(2,i);
       -q(4,i) -q(3,i) -q(2,i)  q(1,i)];

  w(:,i) = 2*W*q_rate(:,i);

end

end

