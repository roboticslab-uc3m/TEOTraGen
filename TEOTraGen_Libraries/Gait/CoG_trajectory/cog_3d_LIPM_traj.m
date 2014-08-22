function [ cog_trajectory ] = cog_3d_LIPM_trajectory(footprints_poses, median_zmp, Ts, TimeSS, TimeDS, zc, alpha, lambda, g)
%COG_3D_LIPM Generates a Center of Gravity (CoG) using the 3D Linear
%Inverted Pendulum Model (LIPM)
%   Input:
%   Output:

Tc = sqrt(zc/g);

timeSS = 0:Ts:TimeSS;

xSS = zeros(size(timeSS));
d_xSS = zeros(size(timeSS));
ySS = zeros(size(timeSS));
d_ySS = zeros(size(timeSS));


% Sagital Movement
x0SS = -lambda;
d_x0SS = -x0SS*(1+cosh(TimeSS/Tc))/(Tc*sinh(TimeSS/Tc)); % Buscar simplificar la formula

% Frontal Movement
y0SS = alpha*cosh(TimeSS/2/Tc);
d_y0SS = -alpha/Tc*sinh(TimeSS/2/Tc);




% Simple Support Trajectory
for kk=1:size(time,2)
    xSS(kk) = xi*cosh((time(kk)-ti)/Tc) + d_xi*Tc*sinh((time(kk)-ti)/Tc);
    d_xSS(kk) = xi/Tc*sinh((time(kk)-ti)/Tc) + d_xi*cosh((time(kk)-ti)/Tc);

     ySS(kk) = yi*cosh((time(kk)-ti)/Tc) + d_yi*Tc*sinh((time(kk)-ti)/Tc);
    d_ySS(kk) = yi/Tc*sinh((time(kk)-ti)/Tc) + d_yi*cosh((time(kk)-ti)/Tc);
end

end

