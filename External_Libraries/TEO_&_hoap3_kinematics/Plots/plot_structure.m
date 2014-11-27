function plot_structure (S, type)
%PLOT_STRUCTURE is a function to plot a plot structure created using
%create_plot_structure.
%
%   The input should be: graph1_data, "graph1_legend",...
%   where graph1_data is an array to be plotted and graph1_legend is its
%   legend for the plot
%
%   See also PLOT, CREATE_PLOT_STRUCTURE.
%
%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2010/03/31 $

time = (0:S.L-1)*S.Ts;
if isfield(S, 'LineSpec')
    plot (time,S.data,S.LineSpec);
else
    plot (time,S.data);
end
if nargin > 1
    if strcmp(type, 'complete')
    xlabel(S.x_lab, 'Interpreter', 'latex', 'FontSize', 12);
    ylabel(S.y_lab, 'Interpreter', 'latex', 'FontSize', 12);
    title(S.name,   'Interpreter', 'latex', 'FontSize', 12);
    end
    if strcmp(type, 'labels')
    xlabel(S.x_lab, 'Interpreter', 'latex', 'FontSize', 12);
    ylabel(S.y_lab, 'Interpreter', 'latex', 'FontSize', 12);
    end
end
