function [ q_rate_dot ] = ang_acc2quat_rate_dot( q, w_dot )
%QUAT_RATE_DOT2ANG_ACC Summary of this function goes here
%   Detailed explanation goes here

% Author: Domingo Esteban

W = [-q(1)  q(0) -q(3)  q(2);
     -q(2)  q(3)  q(0) -q(1);
     -q(3) -q(2) -q(1)  q(0)];

q_rate_dot = 1/2*W'*w_dot;