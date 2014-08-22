function [x_traj, dx_traj, ddx_traj] = poly7_trajectory(P0, P1, Ts)
%POLY7_TRAJECTORY Seventh-order polynomial trajectory interpolation.
%	[X_TRAJ, DX_TRAJ, DDX_TRAJ] = POLY7_TRAJECTORY (P1, P2, TS)
%	provides the trajectory seventh-order polynomial interpolant of the points P1 and P2 with
%	sampling time Ts. P1 and P2 must represent the initial and final
%	conditions. They are struct with position, velocity and acceleration,
%	defined by the function SET_TRAJECTORY_CONDITION.
%
%   Example:
%       P0 = set_trajectory_condition(0, rand(3,1), rand(3,1), rand(3,1));
%       P1 = set_trajectory_condition(2, rand(3,1), rand(3,1), rand(3,1));
%       [x, dx, ddx] = poly7_trajectory (P0, P1, 1e-3);
%

%   See also POLY3_TRAJECTORY, POLY5_TRAJECTORY.

%   Author: Domingo Esteban
%   $Revision: 1.0 $  $Date: 2013/07/23 $


% Avoid possible quantization troubles
t0 = round_to_Ts(P0.t, Ts);
t1 = round_to_Ts(P1.t, Ts);

% Define time vector
tt = 0:Ts:(t1-t0);
L = length(tt);

% Prepare output
n   = size(P0.x, 1);
x   = zeros(L,n);
dx  = zeros(L,n);
ddx = zeros(L,n);

for ii=1:n
    [x(:,ii), dx(:,ii), ddx(:,ii)] = poly7_traj(P0.x(ii), P1.x(ii), tt, P0.dx(ii), P1.dx(ii), P0.ddx(ii), P1.ddx(ii));
end

x_traj = create_trajectory_structure(x,   Ts, tt + t0);
dx_traj = create_trajectory_structure(dx,  Ts, tt + t0);
ddx_traj = create_trajectory_structure(ddx, Ts, tt + t0);



function t_r = round_to_Ts(t, Ts)
t_r = round(t/Ts)*Ts;





