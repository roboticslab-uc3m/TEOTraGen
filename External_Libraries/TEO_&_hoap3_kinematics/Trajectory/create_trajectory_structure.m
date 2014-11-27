function trajectory = create_trajectory_structure(data, Ts, t)
%   CREATE_TRAJECTORY_STRUCTURE creates a structure for some data to be
%   considered as a trajectory.
%   trajectory = create_trajectory_structure(data, Ts, t) creates a structure with
%   the following fields:
%   	trajectory.data = data
%       	data of the trajectory
%       trajectory.Ts = Ts
%       	the sampling time (has to be a number)
%       trajectory.time = t
%       	the corresponding time vector 
%       trajectory.T
%       	the duration of the trajectory
%
%   If the input vector t, constituted by the corresponding time vector, it
%   is not given as input, the function generates a time vector starting
%   from 0.
%
%   Example:
%   	vector = rand (6,1000);
%       trajectory = create_trajectory_structure(vector, 1e-3)

%   Author: Paolo Pierro
%   $Revision: 0.9 $  $Date: 2011/02/02 $

if nargin < 2
    error('CREATE_TRAJECTORY_STRUCTURE:argChk', 'Wrong number of input arguments')
end

[m, n] = size(data);
if nargin < 3
    L_t = max(m, n);
    t = (0:(L_t-1))*Ts;
else
    L_t = length(t);
end

if m == L_t
    data = data';
elseif n ~= L_t
    error('CREATE_TRAJECTORY_STRUCTURE:argChk', 'Wrong type of input arguments: data vector and time vector should have the same length')
end

trajectory.data = data;
trajectory.Ts   = Ts;
trajectory.time = t;
trajectory.T    = t(L_t)-t(1);
end