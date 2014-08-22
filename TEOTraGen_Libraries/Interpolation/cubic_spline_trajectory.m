%***********************************
% CUBIC SPLINE TRAJECTORY
%*********************************
function [x, dx, ddx] = cubic_spline_trajectory (P1,P2,Ts)
T=P2.t-P1.t;
time=0:Ts:T; %T-Ts;
time_phase1 = 0:Ts:T/2;%(T/2-Ts);
time_phase2 = 0:Ts:T/2;%(T/2-Ts);

p=zeros(size(P1.x,1),length(time));

trajectory_mode=ones(size(P1.x,1),1);

for k=1:size(p,1)
    switch trajectory_mode(k)
        case 1
            p(k,:) = interpolation (T, time, P1.x(k), P2.x(k), 0, 0);
        case 0
            p1 = interpolation (T/2, time_phase1, P1.x(k), P2.x(k), 0, 0);
            p2 = interpolation (T/2, time_phase2, P2.x(k), P1.x(k), 0, 0);
            p(k,:) = [p1, p2];
    end
end

tt=time+P1.t;

x  = create_trajectory_structure(p,  Ts, tt);

dp = [zeros(size(p,1),1) diff(p,1,2)];
ddp = [zeros(size(p,1),2) diff(p,2,2)];

dx  = create_trajectory_structure(dp,  Ts, tt);
ddx  = create_trajectory_structure(ddp,  Ts, tt);



% Cubic interpolation taking into account the velocity
function x = interpolation (T, t, x0, x1, v0, v1)
a = x0;
b = v0;
c = (3*(x1-x0) - T*(2*v0+v1))/T^2;
d = (2*(x0-x1) + T*(v0+v1))/T^3;
x = a + b*t + c*t.^2 + d*t.^3;