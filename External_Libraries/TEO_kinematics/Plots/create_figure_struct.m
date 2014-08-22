function S = create_figure_struct (h, folder, name, ext, prefix, id)
%CREATE_FIGURE_STRUCT creates a structure for some data to be plotted
%   S =  CREATE_FIGURE_STRUCT (h, folder, name, ext) creates a structure
%   with the following fields:
%       S.h = data
%           the handle to the figure object
%       S.filename = foldername.ext
%           the full path and name with extension for the output
%   S =  CREATE_FIGURE_STRUCT (h, folder, name, ext, prefix, id) creates
%   a structure with the following fields:
%       S.h = data
%           the handle to the figure object
%       S.filename = foldername_prefix_id.ext
%           the full path and name with extension for the output
%
%   Example:
%       h = plot(rand(5000,1))
%       S =  create_figure_struct (h, '/home/Documents/', 'random', 'bmp')
%
%   See also FIGURE.
%
%   Author: Paolo Pierro $Revision: 1.0 $  $Date: 2010/04/24 $

filename = strcat(folder,name);
if nargin > 4
    filename = strcat(filename,'_');
    filename = strcat(filename,prefix);
    if nargin > 5
        filename = strcat(filename,'_');
        filename = strcat(filename,id);
    end
end
filename = strcat(filename,'.');
filename = strcat(filename,ext);

S.h = h;
S.filename = filename;
