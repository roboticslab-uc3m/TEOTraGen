function [V]=VRow(V)
%
% 
[m,n]=size(V);
if n==1
    V=V';
else
    V=V;
end