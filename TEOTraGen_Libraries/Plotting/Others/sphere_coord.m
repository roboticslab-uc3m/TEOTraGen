function [ x, y, z ] = sphere_coord( r, x0, y0, z0)
%PLOT_SPHERE Returns X, Y and Z of a sphere
%   Detailed explanation goes here
% Author: Domingo Esteban

[x,y,z] = sphere(50);
x = x*r + x0;
y = y*r + y0;
z = z*r + z0;

end

