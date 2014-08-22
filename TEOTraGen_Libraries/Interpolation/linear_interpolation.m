function [x_traj, dx_traj, ddx_traj] = linear_interpolation(P1,P2,Ts)
% LINEAR_INTERPOLATION computes the linear trajectory between two points
%
%   [TM] = linear_interpolation(p0,p1,T,Ts)
%
%   Input:
%       p0: initial point. Insert as row vector
%       p1: end     point. Insert as row vector
%       t0: initial time
%       T:  total time
%       Ts: unit time
%
%   Output:
%       Points: Matrix with trajectory's points as column way
%
%   Example:
%       P1 = set_trajectory_condition(0,rand(3,1), zeros(3,1), zeros(3,1));
%       P2 = set_trajectory_condition(2,rand(3,1), zeros(3,1), zeros(3,1));
%       [x] = lineartrajectory (P1, P2, 1e-3);
%
%   See also SPLINE_INTERPOLATION, SET_TRAJECTORY_CONDITION.

%   Author Daniel GªC Locatelli
%   Adapted by: Domingo Esteban
%   Revision 3.0    Date: 2013/08/20


% Avoid possible quantization troubles
t1 = round_to_Ts(P1.t, Ts);
t2 = round_to_Ts(P2.t, Ts);

% Define time vector
tt = t1:Ts:t2; % instead of 0:Ts:(t2-t1)
L = length(tt);

% Prepare output
n   = size(P1.x, 1);
x   = zeros(n,L);
dx  = zeros(n,L);
ddx = zeros(n,L);

% Convert into a row vector if it is not.
p0=Verify_Row_Vector(P1.x);
p1=Verify_Row_Vector(P2.x);


CM=zeros(2,n);
M=[t1 1;t2 1];
DM=[p0;p1];
for i=1:n
    CM(:,i)=M\DM(:,i);
    x(i,:)=([tt' ones(L,1)]*CM(:,i))';
    dx(i,:)=([ones(L,1) zeros(L,1)]*CM(:,i))';
end

% Define trajectory structures
x_traj    = create_trajectory_structure(x,   Ts, [t1 t2]);
dx_traj   = create_trajectory_structure(dx,  Ts, [t1 t2]);
ddx_traj  = create_trajectory_structure(ddx, Ts, [t1 t2]);
end

function t_r = round_to_Ts(t, Ts)
t_r = round(t/Ts)*Ts;
end

function V = Verify_Row_Vector(V)
% Check if the vector is a row otherwise convert it into one
n=size(V,2);
if n==1
    V = V';
end
end