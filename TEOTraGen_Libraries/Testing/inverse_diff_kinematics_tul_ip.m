% INPUT:
x = operational.trajectory;
xd = operational.d_trajectory;
xdd = operational.dd_trajectory;
q = zeros(6,1);

n = size(q,2);
Q = q(:,end);

for k = 1:kend
  % Reference trajectory of the tip:
  Xref = [x(k) y(k) z(k) theta_roll(k) theta_pitch(k) theta_yaw(k)]';
  Xrefd = [xd(k) yd(k) zd(k) theta_rolld(k) theta_pitchd(k) theta_yawd(k)]';
  Xrefdd = [xdd(k) ydd(k) zdd(k) theta_rolldd(k) theta_pitchdd(k) theta_yawdd(k)]';
  
  % Jacobians:
  J(k) = Jacobian(q(k-1));
  Jd = (J(k) - J(k-1))/delta_t;
  J...
    
% CONTINUARA
  
end