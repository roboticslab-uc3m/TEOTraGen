function result = istrajectory (A)
%ISTRAJECTORY True for trajectories structures.
%   ISTRAJECTORY(A) returns true if A is a structure of type trajectory and
%   false otherwise. 
%
%   Example:
%   	vector = rand (6,1000);
%       trajectory = create_trajectory_structure(vector, 1e-3);
%       istrajectory(trajectory)
%       returns true
%
%   See also CREATE_TRAJECTORY_STRUCTURE, ISFLOAT, ISINTEGER, ISSPARSE,
%   ISLOGICAL, ISCHAR, ISNUMERIC.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/02/02 $

if ~isstruct(A)
    result = false;
    return;
end

if ~isfield(A, 'data')
    result = false;
    return;
end

if ~isnumeric(A.data)
    result = false;
    return;
end

if ~isfield(A, 'time')
    result = false;
    return;
end

if ~isnumeric(A.time)
    result = false;
    return;
end

if ~isfield(A, 'Ts')
    result = false;
    return;
end

if ~isnumeric(A.Ts)
    result = false;
    return;
end

result = true;