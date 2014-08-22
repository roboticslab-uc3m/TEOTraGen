function time = determine_time_vector (M, Ts)
%DETERMINE_TIME_VECTOR determine the time vector corresponding to the input
% data with sampling time Ts.
%
% See also IMPORTLOG, IMPORTDATA, SELECT_TIME_INTERVAL, SELECT_DATA_INTERVAL

% Author: Paolo Pierro $
% $Revision: 0.9 $  $Date: 2010/04/29 $

% Check the number of input arguments
if nargin~=2
    error('DETERMINE_TIME_VECTOR:argChk', 'Wrong number of input arguments')
end

L = size(M,2);
time = (0:L-1)*Ts;