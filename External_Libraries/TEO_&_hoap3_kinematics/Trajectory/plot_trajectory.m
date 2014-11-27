function plot_trajectory (S)
%PLOT_TRAJECTORY is a function to plot a plot structure created using
%create_plot_structure.
%
%   Example:
%   	vector = rand (1,1000);
%       trajectory = create_trajectory_structure(vector, 1e-3);
%       Graph1 = create_plot_structure (trajectory,'[s]','[rad]',...
%       'Graphic 1', '-b');
%       plot_structure(Graph1)
%
%   See also PLOT, CREATE_PLOT_STRUCTURE.

%   Author: Paolo Pierro
%   $Revision: 1.1 $  $Date: 2011/02/02 $

if isfield(S, 'LineSpec')
    plot (S.time,S.data,S.LineSpec);
else
    plot (S.time,S.data);
end
xlabel(S.x_lab);
ylabel(S.y_lab);
title(S.name)