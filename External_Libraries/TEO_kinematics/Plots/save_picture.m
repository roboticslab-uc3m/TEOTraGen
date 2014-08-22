function save_picture (S)
%SAVE_PICTURE exports the figure to the specified file using the specified
% graphics format
%   SAVE_PICTURE (S) saves the vector of structures created by the
%   CREATE_FIGURE_STRUCTURE command.
%
%   See also SAVEAS, CREATE_FIGURE_STRUCT
%
%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2010/04/24 $

if strcmp(input('Save pictures and overwrite previous [y/n]? ', 's'),'y')
    n = size(S,2);
    for jj=1:n
        if exist(S(jj).filename,'file')==2
            delete(S(jj).filename);
        end
        saveas(S(jj).h, S(jj).filename);
    end
end