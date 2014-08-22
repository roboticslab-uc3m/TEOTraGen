function trajectory = combine_trajectories(t1, t2)
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
n = size(t2.data,2);
trajectory.data = [t1.data,t2.data(:,2:n)];
trajectory.Ts   = t1.Ts;
trajectory.time = [t1.time,t2.time(:,2:n)];
trajectory.T    = t1.T+t1.T;
end