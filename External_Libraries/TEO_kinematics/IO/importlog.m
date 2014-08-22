function M_data = importlog (varargin)
%IMPORTLOG Load log data from a file into MATLAB.
%
% A = IMPORTLOG(FILENAME) loads data from FILENAME into A. The data is
% column-wise (transposed with respect to the file)
%
% A = IMPORTLOG(FILENAME, 'DELIMITER', DELIM) loads log from FILENAME using
% DELIM as the column separator. DELIM must be a string. Use '\t' for tab.
%
% A = IMPORTLOG(FILENAME, 'FOLDER', PATHTOFILENAME) loads log from FILENAME
% in the PATHTOFILENAME folder.
%
% A = IMPORTLOG(FILENAME, 'HEADERLINE', HLINE) where HLINE is a number that
% indicates on which line of the file the header text is located, loads
% data from line HLINE+1 to the end of the file.
%
% See also IMPORTDATA 

% Author: Paolo Pierro $
% $Revision: 0.9 $  $Date: 2010/04/28 $

% Check the number of input arguments
error(nargchk(1,7,nargin,'struct'));

% Log file to open
FileName = varargin{1};

% Number of optional input arguments
optargin = size(varargin,2)-1;

% Initializing flags
INPUT_FOLDER = 0;
DELIM_DEF = 0;
HEAD_LIN_DEF = 0;

for jj=2:2:optargin
    if ~ischar(varargin{jj})
        error('IMPORTLOG:argChk', 'Wrong type of input arguments')
    end
    switch lower(varargin{jj})
        case {'folder'}
            folder = varargin{jj+1};
            if ~ischar(folder)
                error ('IMPORTLOG:argChk','Wrong type of input arguments: the parameter of folder must be a string')
            end
            if ~strcmp(folder(length(folder)),'/')
                folder(length(folder)+1) = '/';
            end
            INPUT_FOLDER = 1;
            
        case {'delimiter'}
            delim = varargin{jj+1};
            if ~ischar(delim)
                error ('IMPORTLOG:argChk','Wrong type of input arguments: the parameter of delimiter must be a string')
            end
            DELIM_DEF = 1;
            
        case {'headerline'}
            head_lines = varargin{jj+1};
            if ~isnumeric(head_lines)
                error ('IMPORTLOG:argChk','Wrong type of input arguments: the parameter of headerline must be a number')
            end
            HEAD_LIN_DEF = 1;
    end
end


if INPUT_FOLDER == 1
    % absolute filename (with path)
    FileName = strcat(folder,FileName);
end

if DELIM_DEF == 0
    delim = NaN;
end

if HEAD_LIN_DEF == 0
   head_lines = 0; 
end

% Getting data from file
M =  importdata(FileName, delim, head_lines);
M_data = transpose(M.data);