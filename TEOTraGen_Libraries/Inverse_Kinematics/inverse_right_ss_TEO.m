function [q, dq, ddq] = inverse_right_ss_TEO(q0, trajectory, d_trajectory, dd_trajectory, h)
%
%
% Inverse Kinematics for Right Leg Support in Simple Support
L =length(trajectory.time);
T = trajectory.time(L);
q  = zeros(size(q0,1), L);
dq = zeros(size(q0,1), L);
ddq = zeros(size(q0,1), L);
Ts = trajectory.Ts;
e_RF = zeros(6, L);
e_LF = zeros(6, L);
e_RH = zeros(6, L);
e_LH = zeros(6, L);

% IK constants
K_p = 1e-3;
K_o = pi/8;

Kp = [K_p*eye(3), zeros(3);
      zeros(3),   K_o*eye(3)]/Ts^2;
Kd = [K_p*eye(3), zeros(3);
      zeros(3),   K_o*eye(3)]/Ts;

 
% Initial positions
RF_p0_w = pose_quat2rpy(h.RF_T_w(q0));
LF_p0_w = pose_quat2rpy(h.LF_T_w(q0));
w_p0_LF = pose_quat2rpy(h.w_T_LF(q0));
q(:,1) = q0;
for jj=1:L-1
    % Inverse kinematics for Legs
    switch trajectory.SF(jj)
        case 0      % double support
            e_RF(:,jj) = evaluate_error (trajectory.CoM(:,jj)+RF_p0_w, pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));
            e_LF(:,jj) = evaluate_error (trajectory.CoM(:,jj)+LF_p0_w, pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));
            
            % verificarlo
            u_R = [dd_trajectory.CoM(1:3,jj);zeros(3,1)] + Kd * [d_trajectory.CoM(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
            u_L = [dd_trajectory.CoM(1:3,jj);zeros(3,1)] + Kd * [d_trajectory.CoM(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);
            
            % v = ddx.data(:,jj-1) + Kd * de(:,jj-1) + Kp * e(:, jj-1);
            ddq(1:6, jj+1) = invert_kinematics_standard (q(:,jj), h.RF_J_w, u_R, 1:6, 1:6);
            ddq(7:12, jj+1) = invert_kinematics_standard (q(:,jj), h.LF_J_w, u_L, 1:6, 1:6);          
            
        case -1     % right foot support
%             if trajectory.SF(jj-1)==0
%                 w_p0_LF = pose_quat2rpy(h.w_T_LF(q(:,jj-1)));
%             end
            e_RF(:,jj) = evaluate_error (trajectory.CoM(:,jj)+RF_p0_w, pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));
            e_LF(:,jj) = evaluate_error (trajectory.LF(:,jj)+w_p0_LF, pose_quat2rpy(real(h.w_T_LF(q(:,jj)))));
            % verificarlo
           
            u_R = [dd_trajectory.CoM(1:3,jj);zeros(3,1)] + Kd * [d_trajectory.CoM(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
            u_L = [dd_trajectory.LF(1:3,jj);zeros(3,1)] + Kd * [d_trajectory.LF(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);
           
            ddq(1:6, jj+1) = invert_kinematics_standard (q(:,jj), h.RF_J_w, u_R, 1:6, 1:6);
            ddq(7:12, jj+1) = invert_kinematics_standard (q(:,jj), h.w_J_LF, u_L, 1:6, 1:6);          
            
        case 1      % left foot support
            
            if trajectory.SF(jj-1)==-1
                w_p0_RF = pose_quat2rpy(h.w_T_RF(q(:,jj-1)));
            end
            e_LF(:,jj) = evaluate_error (trajectory.CoM(:,jj)+LF_p0_w, pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));
            e_RF(:,jj) = evaluate_error (trajectory.RF(:,jj)+w_p0_RF, pose_quat2rpy(real(h.w_T_RF(q(:,jj)))));
            % verificarlo
           
            u_L = [dd_trajectory.CoM(1:3,jj);zeros(3,1)] + Kd * [d_trajectory.CoM(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);
            u_R = [dd_trajectory.RF(1:3,jj);zeros(3,1)] + Kd * [d_trajectory.RF(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
           
            ddq(7:12, jj+1) = invert_kinematics_standard (q(:,jj), h.LF_J_w, u_L, 1:6, 1:6);
            ddq(1:6, jj+1) = invert_kinematics_standard (q(:,jj), h.w_J_RF, u_R, 1:6, 1:6);  
            
    end
    % Other joints
    %ddq (13, jj+1) = pi/32/T^2*2;
    
    % Right and left arm
    e_RH(:,jj) = evaluate_error (trajectory.RH(:,jj), pose_quat2rpy(h.CoM_T_RH(q(:,jj))));
    e_LH(:,jj) = evaluate_error (trajectory.LH(:,jj), pose_quat2rpy(h.CoM_T_LH(q(:,jj))));
           
	u_RH = dd_trajectory.RH(:,jj) + Kd * d_trajectory.RH(:,jj) + Kp * e_RH(:,jj);
    u_LH = dd_trajectory.LH(:,jj) + Kd * d_trajectory.LH(:,jj) + Kp * e_LH(:,jj);
           
    ddq(14:18, jj+1) = invert_kinematics_standard (q(:,jj), h.CoM_J_RH, u_RH, 1:3, 1:5);
    ddq(19:23, jj+1) = invert_kinematics_standard (q(:,jj), h.CoM_J_LH, u_LH, 1:3, 1:5);          
    
    dq(:, jj+1)  = integrate_vector (dq(:, jj), ddq(:, jj+1), Ts);
    q(:, jj+1)   = integrate_vector (q(:, jj), dq(:, jj+1), Ts);
end
end


function ddq = invert_kinematics_standard (q_act, J, e, m, n)
J1 = J(q_act);
ddq = J1(m, n)\e(m);
end

function x_next = integrate_vector (x, dx, Ts)
x_next = x + dx * Ts;
end

function error = determine_error (pd, p)
error = pd - p;
end