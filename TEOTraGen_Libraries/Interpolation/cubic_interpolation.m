% Cubic interpolation without taking into account the velocity
function [x, dx, ddx] = cubic_interpolation (P1,P2,Ts)

T=P2.t-P1.t;
time = 0 : Ts : T; %T-Ts

p=zeros(size(P1.x,1),length(time));

for i=1:size(p,1)
    a = -2*(P2.x(i)-P1.x(i))/T^3;
    b = 3*(P2.x(i)-P1.x(i))/T^2;
    p(i,:) = a * time.^3 + b * time.^2 + P1.x(i)*ones(1,length(time));
end

tt=time+P1.t;

x  = create_trajectory_structure(p,  Ts, tt);

dp = [zeros(size(p,1),1) diff(p,1,2)];
ddp = [zeros(size(p,1),2) diff(p,2,2)];

dx  = create_trajectory_structure(dp,  Ts, tt);
ddx  = create_trajectory_structure(ddp,  Ts, tt);

