function error = pose_error (pd, p)

% Position error
error(1:3,:) = pd(1:3) - p(1:3);

% Orientation error (rpy)
error(4:6,:) = pd(4:6) - p(4:6);
end

% function error_o = orientation_error(Q_d, Q)
% %UNTITLED2 Summary of this function goes here
% %   Detailed explanation goes here
% eta = Q(1);
% eps = Q(2:4);
% 
% eta_d = Q_d(1);
% eps_d = Q_d(2:4);
% 
% error_o = eta*eps_d - eta_d*eps - matrix_S(eps_d)*eps;
% end
% 
% function S = matrix_S (w)
% S = [       0,  -w(3),   w(2);
%          w(3),      0,  -w(1);
%         -w(2),   w(1),     0];
% end