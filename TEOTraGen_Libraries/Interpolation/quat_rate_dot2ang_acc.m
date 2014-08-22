function [ w_dot ] = quat_rate_dot2ang_acc( q, q_rate_dot )
%QUAT_RATE_DOT2ANG_ACC Summary of this function goes here
%   Detailed explanation goes here

% Author: Domingo Esteban

  nelements = size(q,2);
  w_dot = zeros(3,nelements);

  for i=1:nelements,

    W = [-q(2,i)  q(1,i) -q(4,i)  q(3,i);
         -q(3,i)  q(4,i)  q(1,i) -q(2,i);
         -q(4,i) -q(3,i) -q(2,i)  q(1,i)];

    w_dot(:,i) = 2*W*q_rate_dot(:,i);
  end
end

