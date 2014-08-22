function [x_traj, dx_traj, ddx_traj] = poly3_viapoints_trajectory(P, Ts)
%POLY3_VIAPOINTS_TRAJECTORY Third-order polynomial trajectory interpolation
%   specifying a sequence of desired points (via-points) without indication
%   on the velocity in these points.
%	[X_TRAJ, DX_TRAJ, DDX_TRAJ] = POLY3_VIAPOINTS_TRAJECTORY (P, TS)
%	provides the trajectory third-order polynomial interpolant of the n via-points P with
%	sampling time Ts. P must represent a (1xN) vector of Points extructures
%   They are struct with position, velocity and acceleration,
%	defined by the function SET_TRAJECTORY_CONDITION.
%
%   Example:
%       P0 = set_trajectory_condition(0, rand(3,1), zeros(3,1), zeros(3,1));
%       P1 = set_trajectory_condition(2, rand(3,1), zeros(3,1), zeros(3,1));
%       P2 = set_trajectory_condition(5, rand(3,1), zeros(3,1), zeros(3,1));
%       P  = [P0 P1 P2];
%       [x, dx, ddx] = poly3_viapoints_trajectory (P, 1e-3);
%
%   Note: With a Third-order polynomial trajectory with via-points is not possible to
%   determine initial and final velocities and accelerations. So the function will ignore
%   them and it will consider those that allows to obtain the desired initial and final
%   positions.
%
%   See also POLY5_VIAPOINTS_TRAJECTORY, POLY7_VIAPOINTS_TRAJECTORY, POLY3_TRAJ.

%   Author: Domingo Esteban
%   $Revision: 1.0 $  $Date: 2013/07/24 $

num_points = size(P,2);

if num_points<2
    error('Must specify two points or more');
end

n   = size(P(1).x, 1);

for ii=1:num_points
    for kk=1:n
        if (ii==1 || ii==num_points)
            P(ii).dx(kk)=0;
        else
            v1=(P(ii).x(kk)-P(ii-1).x(kk))/(P(ii).t-P(ii-1).t);
            v2=(P(ii+1).x(kk)-P(ii).x(kk))/(P(ii+1).t-P(ii).t);
            if sign(v1)==sign(v2)
                P(ii).dx(kk) = (v1+v2)/2;
            else
                P(ii).dx(kk) = 0;
            end
        end
        P(ii).ddx(kk)=0;
    end
end

[x_traj, dx_traj, ddx_traj] = poly3_segments_trajectory(P, Ts);
