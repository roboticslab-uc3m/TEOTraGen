function [ Ta ] = Tanalytical( rpy )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

T = [cos(rpy(2))*cos(rpy(3))    -sin(rpy(3))    0; 
    cos(rpy(2))*sin(rpy(3))     cos(rpy(3))     0; 
    -sin(rpy(2))                0               1];

Ta=[eye(3,3) zeros(3,3); zeros(3,3) T];

end

