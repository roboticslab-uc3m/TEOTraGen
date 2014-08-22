function [x_traj, dx_traj, ddx_traj] = poly5_segments_trajectory(P, Ts)
%POLY5_SEGMENTS_TRAJECTORY Fifth-order polynomial trajectory interpolation for n segments.
%	[X_TRAJ, DX_TRAJ, DDX_TRAJ] = POLY5_SEGMENTS_TRAJECTORY (P, TS)
%	provides the trajectory fifth-order polynomial interpolant of the n segments of points P with
%	sampling time Ts. P must represent a (1xN) vector of Points extructures
%   They are struct with position, velocity and acceleration,
%	defined by the function SET_TRAJECTORY_CONDITION.
%
%   Example:
%       P0 = set_trajectory_condition(0, rand(3,1), rand(3,1), rand(3,1));
%       P1 = set_trajectory_condition(2, rand(3,1), rand(3,1), rand(3,1));
%       P2 = set_trajectory_condition(5, rand(3,1), rand(3,1), rand(3,1));
%       P  = [P0 P1 P2];
%       [x, dx, ddx] = poly5_segments_trajectory (P, 1e-3);
%
%   See also POLY3_SEGMENTS_TRAJECTORY, POLY7_SEGMENTS_TRAJECTORY, POLY5_TRAJ.

%   Author: Domingo Esteban
%   $Revision: 1.0 $  $Date: 2013/07/24 $

num_points = size(P,2);

if num_points<2
    error('Must specify two points or more');
end

for ii=1:num_points-1
    
    % Avoid possible quantization troubles
    t0 = round_to_Ts(P(ii).t, Ts);
    t1 = round_to_Ts(P(ii+1).t, Ts);

    % Define time vector
    tt = 0:Ts:roundn(t1-t0,-10); % We round this value to solve floating point accuracy problem
    L = length(tt);

    % Prepare output
    n   = size(P(ii).x, 1);
    x   = zeros(L,n);
    dx  = zeros(L,n);
    ddx = zeros(L,n);

    for kk=1:n
        [x(:,kk), dx(:,kk), ddx(:,kk)] = poly5_traj(P(ii).x(kk), P(ii+1).x(kk), tt, P(ii).dx(kk), P(ii+1).dx(kk), P(ii).ddx(kk), P(ii+1).ddx(kk));
    end

    x_segm_traj = create_trajectory_structure(x,   Ts, tt + t0);
    dx_segm_traj = create_trajectory_structure(dx,  Ts, tt + t0);
    ddx_segm_traj = create_trajectory_structure(ddx, Ts, tt + t0);
    
    if ii==1
        x_traj = x_segm_traj;
        dx_traj = dx_segm_traj;
        ddx_traj = ddx_segm_traj;
    else
        x_traj = combine_2_traj(x_traj, x_segm_traj);
        dx_traj = combine_2_traj(dx_traj, dx_segm_traj);
        ddx_traj = combine_2_traj(ddx_traj, ddx_segm_traj);
    end
end


function t_r = round_to_Ts(t, Ts)
t_r = round(t/Ts)*Ts;


function combined_traj = combine_2_traj(traj1,traj2)
    combined_traj.data = [traj1.data(:,1:end-1) traj2.data(:,:)];
    combined_traj.Ts =  traj1.Ts;
    combined_traj.time = [traj1.time(:,1:end-1) traj2.time(:,:)];
    combined_traj.T = traj1.T+traj2.T;