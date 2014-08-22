function [q, q_dot, p, R, error_p, error_o] = inverse_kinematics_leftALPHA (Ts, T, q, q_dot, Kp, Ko,  xd, xd_dot, Rd, wd, h, SF)

% Initialization
iterations = round_to_Ts(T/Ts,Ts);
error_p = zeros(3,iterations);
error_o = zeros(3,iterations);
R = zeros(4,iterations);
p = zeros(3,iterations);

% Choose the correct formula
if strcmp(SF,'Right')
    DK = h.w_T_LF;
    Jacobian = h.w_J_LF;
elseif strcmp(SF,'Left')
    DK = h.LF_T_w;
    Jacobian = h.LF_J_w;
else
    error('Wrong Support Foot option');
end

for kk = 1:iterations-1
    %desired quaternion
    eta_d = Rd(1,kk);
    eps_d = Rd(2:4,kk);
    %present position
    pose = DK(q(:,kk));
    p(:,kk) = pose(1:3);
    R(:,kk) = pose(4:7);
    %present quaternion
    eta = R(1,kk);
    eps = R(2:4,kk);
    %errors
    error_p(:,kk) = xd(:,kk) - p(:,kk);
    error_o(:,kk) = eta*eps_d - eta_d*eps - matrix_S(wd(:,kk))*eps;
    %present velocity
    q_dot(7:12,kk) = Jacobian(q(:,kk))\[xd_dot(:,kk) + Kp*error_p(:,kk); wd(:,kk) + Ko*error_o(:,kk)];
    %next q
    q(7:12,kk+1) = q(7:12,kk) + q_dot(7:12,kk)*Ts;
end
pose = DK(q(:,iterations));
R(:,iterations) = pose(4:7);
p(:,iterations) = pose(1:3);
eta = R(1,iterations);eps = R(2:4,iterations);
error_p(:,iterations) = xd(:,iterations) - p(:,iterations);
error_o(:,iterations) = eta*eps_d - eta_d*eps - matrix_S(wd(:,iterations))*eps;


function S = matrix_S (w)
S = [       0,  -w(3),   w(2);
         w(3),      0,  -w(1);
        -w(2),   w(1),     0];
