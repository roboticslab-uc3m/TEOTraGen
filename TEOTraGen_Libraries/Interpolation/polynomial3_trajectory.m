function [x_traj, dx_traj, ddx_traj] = polynomial3_trajectory(P0, P1, Ts)
% POLY3_TRAJ computes the polinomic trajectory between two points
%
%   [Points] = poly3_traj(p0,p1,v0,v1,T,Ts,caso)
%
%   Input:
%       p0: initial point. Insert as row vector
%       p1: end     point. Insert as row vector
%       v0: initial velocity
%       v1: final velocity
%       t0: initial time
%       T:  total time
%       Ts: unit time
%
%   Output:
%       Points: Matrix with trajectory's points as column way
%       VPoints: Matrix with velocities
%       APoints: Matrix with
%
%   Example:
%       P0 = set_trajectory_condition(0, rand(3,1), rand(3,1), rand(3,1));
%       P1 = set_trajectory_condition(2, rand(3,1), rand(3,1), rand(3,1));
%       [x, dx, ddx] = poly3_traj (P0, P1, 1e-3);
%
%   Author Daniel G�C Locatelli
%   Adapted: Domingo Esteban
%   Revision 1.0    Date: 2010/02/16

% (p0,p1,v0,v1,t0,T,Ts)

t1 = round_to_Ts(P0.t, Ts);
t2 = round_to_Ts(P1.t, Ts);

p0 = P0.x; p1 = P1.x;
v0 = P0.dx;v1 = P1.dx;
t0 = P0.t; T = P1.t;
number_values=size(P0.x,1);
p0=VRow(p0);
p1=VRow(p1);
v0=VRow(v0);
v1=VRow(v1);
n=round((t2-t1)/Ts)+1;
Points=zeros(3,n);
VPoints=zeros(3,n);
APoints=zeros(3,n);
t=t1:Ts:t2;
CM=zeros(4,6);
M=[t0^3 t0^2 t0 1;T^3 T^2 T 1;3*t0^2 2*t0 1 0;3*T^2 2*T 1 0];
MI=inv(M);
DM=[p0;p1;v0;v1];
for i=1:number_values
    CM(:,i)=MI*DM(:,i);
    Points(i,:)=[[[t.^3]', [t.^2]', t', ones(n,1)]*CM(:,i)]';
    VPoints(i,:)=[[[3*t.^2]' [2*t]' ones(n,1) zeros(n,1)]*CM(:,i)]';
    APoints(i,:)=[[[6*t]' 2*ones(n,1) zeros(n,1) zeros(n,1)]*CM(:,i)]';
end

tt = t1:Ts:t2;
x_traj    = create_trajectory_structure(Points, Ts,tt);
dx_traj   = create_trajectory_structure(VPoints,  Ts,tt);
ddx_traj  = create_trajectory_structure(APoints, Ts,tt);

function t_r = round_to_Ts(t, Ts)
t_r = round(t/Ts)*Ts;




