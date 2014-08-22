function [ quat_rate, quat_rate2 ] = quat_rate( quaternions, Ts )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if size(quaternions,2) == 2,
  quat_rate = (1/Ts) * diff(quaternions,1,2);
  quat_rate2 = 0;
%   quat_rate2 = (1/Ts) * diff(quat_rate,1,2);
else
quat_rate = (1/Ts) * [diff(quaternions,1,2) zeros(4,1)];

quat_rate2 = (1/Ts) * [diff(quat_rate,1,2) zeros(4,2)];
end
end

