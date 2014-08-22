function [x, dx, ddx] = lineartrajectory(P1,P2,Ts)
%
% LINEAR_TRAJECTORY computes the linear trajectory between two points
%
%   [TM] = linear_trajectory(p0,p1,T,Ts)
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
%       P1 = set_trajectory_condition(0,rand(3,1), zeros(3,1), zeros(3,1));
%       P2 = set_trajectory_condition(2,rand(3,1), zeros(3,1), zeros(3,1));
%       [x] = lineartrajectory (P1, P2, 1e-3);

%   Author: Domingo Esteban
%   References: Daniel GªC Locatelli
%   
%   Revision 1.0    Date: 2013/07/14

% Avoid possible quantization troubles
t1 = round_to_Ts(P1.t, Ts);
t2 = round_to_Ts(P2.t, Ts);

% Define time vector
tt = t1:Ts:t2;% instead of 0:Ts:(t2-t1)
L = length(tt);

% Prepare output
n   = size(P1.x, 1);
x   = zeros(n,L);
dx  = zeros(n,L);
ddx = zeros(n,L);


p0=VRow(P1.x);
p1=VRow(P2.x);


CM=zeros(2,n);
M=[t1 1;t2 1];
DM=[p0;p1];
for i=1:n
    CM(:,i)=M\DM(:,i);
    x(i,:)=([tt' ones(L,1)]*CM(:,i))';
    dx(i,:)=([ones(L,1) zeros(L,1)]*CM(:,i))';
end

% x(:,end)= x(:,end-1)+diff(x(:,end-2:end-1),1,2);
% dx(:,end)= dx(:,end-1)+diff(dx(:,end-2:end-1),1,2);
% ddx(:,end)= ddx(:,end-1)+diff(ddx(:,end-2:end-1),1,2);

x  = create_trajectory_structure(x,  Ts, tt);
dx = create_trajectory_structure(dx, Ts, tt);
ddx = create_trajectory_structure(ddx, Ts, tt);

end

function t_r = round_to_Ts(t, Ts)
t_r = round(t/Ts)*Ts;
end
