function [x_traj, dx_traj, ddx_traj, C, r, plane, n] = arc_trajectory3(A,B,C,Ts)
% ARC_TRAJECTORY computes the arc of circumference between 3 points
%
% [C, R, plane, n, P] = arc_trajectory(p1,p2,p3)
%
%   Input:
%       p1: initial point. As a row vector
%       p2: intermediate point. As a row vector
%       p3: final point. As a row vector
%       T:  total time
%       Ts: unit time
%
%   Output:
%       C: the centre of this arc
%       r: the radius of this arc
%       plane: plane made by the three points
%       n: normal vector of the plane
%       P: arc's points
%       initial_points
%
%   Author Daniel GªC Locatelli
%   Revision 3.0    Date: 2010/07/6

% (p1,p2,p3,Ti,Tf,Ts)

p1= A.x;p2 =B.x;p3 =C.x;
Ti = A.t; Tf = C.t;

try
    p1=VRow(p1);p2=VRow(p2);p3=VRow(p3);
    [C,r,plane,n]=circum(p1,p2,p3);
    %initial_points=([p1;p2;p3;C])';
    n=eval(n)/norm(eval(n));
    u=(p1-C)/norm(p1-C);
    v=cross(n,u);
    P1=p1-C;
    P2=p2-C;
    P3=p3-C;
    alfa=acos(P1*P3'/(norm(P1)*norm(P3)));
    beta=acos(P1*P2'/(norm(P1)*norm(P2)));
    if alfa <= beta
        alfa=2*pi-alfa;
    end
    delta_s = alfa*Ts/(Tf-Ti);
    s=0:delta_s:alfa;
    ds = delta_s/Ts;
    x = (r*(cos(s')*u+sin(s')*v)+ones(size(s'))*C)';
    dx = r*ds*(cos(s')*v-sin(s')*u);
    ddx = r*ds^2*(-sin(s')*v-cos(s')*u);
    
    x_traj    = create_trajectory_structure(x, Ts, [Ti Tf]);
    dx_traj   = create_trajectory_structure(dx', Ts, [Ti Tf]);
    ddx_traj  = create_trajectory_structure(ddx', Ts, [Ti Tf]);
    
catch
    
    C=0;n=0;r=0;plane=0;x_traj=0;dx_traj = 0;ddx_traj=0;
%    initial_points=([p1;p2;p3;C])';
    disp('It doesnt exist the arc between these three points');
    return;
end




function [C,r,plane,n]=circum(p1,p2,p3)
%
% FUNCTION TO WORK OUT Center, Radius and Plane
%

% Matrix of points
MP = [p1;p2;p3];
%Matrix of medium points
MPM = [p1 + (p2-p1)/2;p3 + (p2-p3)/2];

% Plane of 3 points
syms x y z real
vsym = [x y z];
plane = det([vsym-p1;p2-p1;p3-p1]);

% normal vector of the plane
 n=[diff(plane,x) diff(plane,y) diff(plane,z)];
 
% directional vectors
v=[p2-p1;p2-p3];

% Matrix of p2 vs p1
MVi0=[n;v(1,:)];

% Matriz of p2 vs p3

MVi1=[n;v(2,:)];

% Normal vector to n and v1
syms x1 y1 z1 real
vsym1=[x1 y1 z1];
Sistema1=MVi0*vsym1';
% Solution system1
x1=1;
[sol1,sol2]=solve(eval(Sistema1));
vp3=[1 eval(sol1) eval(sol2)];


% Normal vector to n and v2
syms x2 y2 z2 real
vsym2=[x2 y2 z2];
Sistema2=MVi1*vsym2';
% Solution system2
x2=1;
[sol3,sol4]=solve(eval(Sistema2)); 
vp2=[1 eval(sol3) eval(sol4)];

MVP=[vp3;vp2];
% Solution of centre
syms r s real
Sistema3=MPM(1,:)+r*MVP(1,:)-MPM(2,:)-s*MVP(2,:);
[r,s]=solve(Sistema3(1),Sistema3(2));
C=eval(sym(MPM(1,:)+r*MVP(1,:)));

% Matrix with distance point vs centre
MR=[norm(MP(1,:)-C);norm(MP(2,:)-C);norm(MP(3,:)-C)];
r=MR(1);