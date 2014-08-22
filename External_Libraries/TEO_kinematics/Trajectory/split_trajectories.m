function S = split_trajectories (traj)
%SPLIT_TRAJECTORIES splits an n-dimensional trajectory in n one-dimensional
%trajectories.
%   S = SPLIT_TRAJECTORIES (TRAJ) splits the n-dimensional trajectory TRAJ
%   created by create_trajectory_structure in n one-dimensional
%   trajectories S which have the same time conditions.
%
%   See also CREATE_TRAJECTORY_STRUCTURE.

%   Author: Paolo Pierro
%   $Revision: 0.9 $  $Date: 2011/02/04 $
[a, b] = size(traj.data);
for jj=1:a
    S(jj) = create_trajectory_structure(traj.data(jj,:), traj.Ts, traj.time);
end
end