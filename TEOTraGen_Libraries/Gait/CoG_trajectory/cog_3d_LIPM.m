function [ traj, dtraj ] = cog_3d_LIPM( lambdaI, lambdaF, betaI, betaF, zc, g, T, Ts)
%cog_3d_LIPM Summary of this function goes here

% Author: Domingo Esteban

Tc = sqrt(zc/g);

time = 0:Ts:T;

x = zeros(size(time));
d_x = zeros(size(time));
y = zeros(size(time));
d_y = zeros(size(time));
z = zc*ones(size(time));
d_z = zeros(size(time));


% Sagital Movement
% x0 = -lambda;
% d_x0 = -x0*(1+cosh(T/Tc))/(Tc*sinh(T/Tc)); % Buscar simplificar la formula
x0 = lambdaI;
d_x0 = (lambdaF - lambdaI*cosh(T/Tc))/(Tc*sinh(T/Tc));

% Frontal Movement
% y0 = direction*beta*cosh(T/2/Tc);
% d_y0 = -direction*beta/Tc*sinh(T/2/Tc);
y0 = betaI;
d_y0 = (betaF - betaI*cosh(T/Tc))/(Tc*sinh(T/Tc));


for kk = 1:size(time,2)
  x(kk) = x0*cosh((time(kk)-time(1))/Tc) + d_x0*Tc*sinh((time(kk)-time(1))/Tc);
  d_x(kk) = x0/Tc*sinh((time(kk)-time(1))/Tc) + d_x0*cosh((time(kk)-time(1))/Tc);

   y(kk) = y0*cosh((time(kk)-time(1))/Tc) + d_y0*Tc*sinh((time(kk)-time(1))/Tc);
  d_y(kk) = y0/Tc*sinh((time(kk)-time(1))/Tc) + d_y0*cosh((time(kk)-time(1))/Tc);
end

traj = [x; y; z];
dtraj = [d_x; d_y; d_z];

end

