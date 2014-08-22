function [ sphere_plotted ] = plot_sphere( r, x0, y0, z0, color )
%PLOT_SPHERE Plots a sphere of radius r in position x y z
%   Detailed explanation goes here
% Author: Domingo Esteban

[x,y,z] = sphere(50);
x = x*r + x0;
y = y*r + y0;
z = z*r + z0;

sphere_plotted = surface(x,y,z,'FaceColor', color, 'EdgeColor', 'none');

end

