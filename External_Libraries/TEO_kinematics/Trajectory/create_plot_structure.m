function S = create_plot_structure (traj, x_lab, y_lab, name, LineSpec)
%CREATE_PLOT_STRUCTURE creates a structure for some data to be plotted
%   S =  create_plot_structure (traj, x_lab, y_lab, name, Ts, LineSpec,
%   LineColor) creates a structure with the following fields:
%       S.data = traj
%           the data to be plotted (has to be a number)
%       S.L = length (traj)
%           the number of elements of data
%       S.x_lab = x_lab
%           the label for x-axis (has to be a string)
%       S.y_lab = y_lab
%           the label for y-axis (has to be a string)
%       S.name = name
%           the name to be put in the plot (has to be a string)
%       S.Ts = Ts
%           the sampling time (has to be a number)
%
%       OPTIONAL: S.LineSpec = LineSpec
%           the line style for the plot (has to be a string). Check Matlab help
%           for more information on specifying line styles and colors.
%
%
%   Example:
%   	vector = rand (1,1000);
%       trajectory = create_trajectory_structure(vector, 1e-3)
%       Graph1 = create_plot_structure (trajectory,'[s]','[rad]','Graphic 1', '-b');
%
%   See also PLOT.

%   Author: Paolo Pierro
%   $Revision: 1.3 $  $Date: 2011/02/02 $

if nargin < 4
    error('CREATE_PLOT_STRUCTURE:argChk', 'Wrong number of input arguments')
end
if ~istrajectory(traj)
    error('CREATE_PLOT_STRUCTURE:argChk', 'Wrong type of input arguments: traj should be a trajectory')
end
if ~ischar(x_lab)
    error('CREATE_PLOT_STRUCTURE:argChk', 'Wrong type of input arguments: x_lab should be a string')
end
if ~ischar(y_lab)
    error('CREATE_PLOT_STRUCTURE:argChk', 'Wrong type of input arguments: y_lab should be a string')
end
if ~ischar(name)
    error('CREATE_PLOT_STRUCTURE:argChk', 'Wrong type of input arguments: name should be a string')
end

S = traj;
S.L = length (traj.data);
S.x_lab = x_lab;
S.y_lab = y_lab;    
S.name = name;

if nargin > 4
    if ~ischar(LineSpec)
        error('CREATE_PLOT_STRUCTURE:argChk', 'Wrong type of input arguments: LineSpec should be a string')
    end
    S.LineSpec = LineSpec;
end