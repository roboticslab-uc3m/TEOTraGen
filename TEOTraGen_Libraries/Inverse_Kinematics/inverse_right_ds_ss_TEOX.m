function [q, dq, ddq] = inverse_right_ds_ss_TEOX(q0, trajectory, d_trajectory, dd_trajectory, h, TEO)

%INVERSE_RIGHT_DS_SS_TEO Inverse Differential Kinematics Algorithm with right foot as support for the robot TEO
%   [Q, DQ, DDQ] = 
%   INVERSE_RIGHT_DS_SS_TEO(Q0, TRAJECTORY, D_TRAJECTORY, DD_TRAJECTORY, H)
%   returns the joint trajectory, joint velocity and joint acceleration for
%   the selected trajectory.
%
%   INPUT:
%       Q0 = 
%       TRAJECTORY = 
%       D_TRAJECTORY = 
%       DD_TRAJECTORY = 
%
%   See also INVERSE_LEFT_DS_SS_TEO.

%   Author: Domingo Esteban
%   References from: P. Pierro and D. García-Cano
%   RoboticsLab - Universidad Carlos III de Madrid
%   $Revision: 1.0 $  $Date: 2013/08/05 $
% *************************************************************************

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%%     INVERSE_RIGHT_DS_SS_TEO    %%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inverse Kinematics for Right Leg Support in Simple Support

% Parameters and Variables needed
    L = length(trajectory.time);
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
    K_p = 100e-3; % Position Gain --> ANTES: 5e-3
    K_o = pi/8; % Orientation Gain --> ANTES: 2*pi/8

% Proportional and Derivative Gain
%     Kp = [K_p*eye(3), zeros(3);
%           zeros(3),   K_o*eye(3)]/Ts^2;
%     Kd = [K_p*eye(3), zeros(3);
%           zeros(3),   K_o*eye(3)]/Ts;
Kp = 5*eye(3);
Ko = 5*eye(3);

 
% Initial pose   
    
    q(:,1) = q0;
    change=0;
    
    pose_SF_RF = trajectory.RF(:,1);
    pose_SF_LF = trajectory.LF(:,1);
    CoM_p0_RH = pose_quat2rpy(h.CoM_T_RH(q(:,1)));
    CoM_p0_LH = pose_quat2rpy(h.CoM_T_LH(q(:,1)));
    ERROR_pose_LF = trajectory.LF(:,1)-(pose_quat2rpy(h.w_T_LF(q0))+(pose_SF_RF+pose_quat2rpy(h.RF_T_w(q0))));
    
for jj=1:L-1
    % Inverse kinematics for Legs
    switch trajectory.SF(jj)
        case 0      % double support
            if change==1
                LF_p0_w = pose_quat2rpy(h.LF_T_w(q(:,jj)));
                difference_w = trajectory.CoM(:,jj-1);
            end
%             e_RF(:,jj) = evaluate_error (trajectory.CoM(:,jj)-pose_quat2rpy(h.w_T_CoM(q(:,jj))), pose_SF_RF+pose_quat2rpy(h.RF_T_w(q(:,jj))));  % Error Right Foot pose
%             e_LF(:,jj) = evaluate_error (trajectory.CoM(:,jj)-pose_quat2rpy(h.w_T_CoM(q(:,jj))), pose_SF_LF+pose_quat2rpy(h.LF_T_w(q(:,jj))));  % Error Left Foot pose
            [e_p_RF(:,jj) e_o_RF(:,jj)] = determine_error (trajectory.CoM(:,jj)-pose_quat2rpy(h.w_T_CoM(q(:,jj))),...
                   pose_SF_RF+pose_quat2rpy(h.RF_T_w(q(:,jj))));              
            [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (trajectory.CoM(:,jj)-pose_quat2rpy(h.w_T_CoM(q(:,jj))),...
                   pose_SF_LF+pose_quat2rpy(h.LF_T_w(q(:,jj))));


            % verificarlo
%             u_R = [dd_trajectory.CoM(1:3,jj);zeros(3,1)] + Kd * [d_trajectory.CoM(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);  % Control signal right
%             u_L = [dd_trajectory.CoM(1:3,jj);zeros(3,1)] + Kd * [d_trajectory.CoM(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);  % Control signal left
            u_R = [Kp*e_p_RF(:,jj); Ko*e_o_RF(:,jj)];
            u_L = [Kp*e_p_LF(:,jj); Ko*e_o_LF(:,jj)];
            
            % v = ddx.data(:,jj-1) + Kd * de(:,jj-1) + Kp * e(:, jj-1);
            dq(1:6, jj+1) = invert_kinematics_standard (q(:,jj), h.RF_J_w, u_R, 1:6, 1:6);
            dq(7:12, jj+1) = invert_kinematics_standard (q(:,jj), h.LF_J_w, u_L, 1:6, 1:6);  
            
            change = 0;
        case -1     % right foot support
%             if change==0
%                 CoM_RF = pose_SF_RF+pose_quat2rpy(h.RF_T_w(q(:,jj-1)))+pose_quat2rpy(h.w_T_CoM(q(:,jj-1)))-difference_w_CoM_RF;
%                 error_p0_LF = CoM_RF + pose_quat2rpy(h.CoM_T_LF(q(:,jj-1)))- pose_SF_RF;
%             end
%             e_RF(:,jj) = evaluate_error (trajectory.CoM(:,jj), pose_SF_RF+pose_quat2rpy(h.RF_T_w(q(:,jj)))+pose_quat2rpy(h.w_T_CoM(q(:,jj)))-difference_w_CoM_RF);
%             e_LF(:,jj) = evaluate_error (trajectory.LF(:,jj) + error_p0_LF, pose_SF_RF+pose_quat2rpy(h.RF_T_w(q(:,jj)))+pose_quat2rpy(h.w_T_CoM(q(:,jj)))-difference_w_CoM_RF + pose_quat2rpy(h.w_T_LF(q(:,jj))));
              [e_p_RF(:,jj) e_o_RF(:,jj)] = determine_error (trajectory.CoM(:,jj)-pose_quat2rpy(h.w_T_CoM(q(:,jj))),...
                   pose_SF_RF+pose_quat2rpy(h.RF_T_w(q(:,jj))));
%               [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (zeros(6,1),zeros(6,1));
%               [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (trajectory.LF(:,jj),...
%                    (pose_SF_RF+pose_quat2rpy(h.RF_T_w(q(:,jj))))+pose_quat2rpy(h.w_T_LF(q(:,jj))));
            % verificarlo

%             u_R = [dd_trajectory.CoM(1:3,jj);zeros(3,1)] + Kd * [d_trajectory.CoM(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
%             u_L = [dd_trajectory.LF(1:3,jj);zeros(3,1)] + Kd * [d_trajectory.LF(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);
            u_R = [Kp*e_p_RF(:,jj); Ko*e_o_RF(:,jj)];
%             u_L = [Kp*e_p_LF(:,jj); Ko*e_o_LF(:,jj)];
           
            dq(1:6, jj+1) = invert_kinematics_standard (q(:,jj), h.RF_J_w, u_R, 1:6, 1:6);
%             ddq(7:12, jj+1) = invert_kinematics_standard (q(:,jj), h.w_J_LF, u_L, 1:6, 1:6);          

            change=1;
    end
    
    % Other joints
    %ddq (13, jj+1) = pi/32/T^2*2;
    
    % Right and left arm
    if jj>1
        CoM_p0_RH = pose_quat2rpy(h.CoM_T_RH(q(:,jj-1)));
        CoM_p0_LH = pose_quat2rpy(h.CoM_T_LH(q(:,jj-1)));
    end
    
%     e_RH(:,jj) = evaluate_error (trajectory.RH(:,jj)+CoM_p0_RH, pose_quat2rpy(h.CoM_T_RH(q(:,jj))));
%     e_LH(:,jj) = evaluate_error (trajectory.LH(:,jj)+CoM_p0_LH, pose_quat2rpy(h.CoM_T_LH(q(:,jj))));
      [e_p_RH(:,jj) e_o_RH(:,jj)] = determine_error (zeros(6,1), zeros(6,1));
      [e_p_LH(:,jj) e_o_LH(:,jj)] = determine_error (zeros(6,1), zeros(6,1));
           
% 	u_RH = dd_trajectory.RH(:,jj) + Kd * d_trajectory.RH(:,jj) + Kp * e_RH(:,jj);
%     u_LH = dd_trajectory.LH(:,jj) + Kd * d_trajectory.LH(:,jj) + Kp * e_LH(:,jj);
            u_RH = [Kp*e_p_RH(:,jj); Ko*e_o_RH(:,jj)];
            u_LH = [Kp*e_p_LH(:,jj); Ko*e_o_LH(:,jj)];
           
%     ddq(15:20, jj+1) = invert_kinematics_standard (q(:,jj), h.CoM_J_RH, u_RH, 1:3, 1:6);
%     ddq(21:26, jj+1) = invert_kinematics_standard (q(:,jj), h.CoM_J_LH, u_LH, 1:3, 1:6);          
    dq(15:20, jj+1) = invert_kinematics_standard (q(:,jj), h.CoM_J_RH, u_RH, 1:3, 1:6);
    dq(21:26, jj+1) = invert_kinematics_standard (q(:,jj), h.CoM_J_LH, u_LH, 1:3, 1:6);      
    
%     dq(:, jj+1)  = integrate_vector (dq(:, jj), ddq(:, jj), Ts);
    q(:, jj+1)   = integrate_vector (q(:, jj), dq(:, jj), Ts);
    
    
    if (trajectory.SF(jj)== -1)
       [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (trajectory.LF(:,jj),...
            (pose_SF_RF+pose_quat2rpy(h.RF_T_w(q(:,jj))))+pose_quat2rpy(h.w_T_LF(q(:,jj))));
        u_L = [Kp*e_p_LF(:,jj); Ko*e_o_LF(:,jj)];
        dq(7:12, jj+1) = invert_kinematics_standard (q(:,jj), h.w_J_LF, u_L, 1:6, 1:6);
        q(7:12, jj+1)   = integrate_vector (q(7:12, jj), dq(7:12, jj), Ts);
    end

end
end


function ddq = invert_kinematics_standard (q_act, J, e, m, n)
J1 = J(q_act);
ddq = J1(m, n)\e(m); %A\B matrix division of A into B. Same as INV(A)*B
end

function x_next = integrate_vector (x, dx, Ts)
x_next = x + dx * Ts;
end

function [error_p error_o] = determine_error (pd, p)
deseado = pose_rpy2quat(pd);
real = pose_rpy2quat(p);
eta = real(4);
eps = real(5:end);
eta_d = deseado(4);
eps_d = deseado(5:end);

error_p= deseado(1:3) - real(1:3);
error_o = eta*eps_d - eta_d*eps - matrix_S(eps_d)*eps;
end

function S = matrix_S (w)
S = [       0,  -w(3),   w(2);
         w(3),      0,  -w(1);
        -w(2),   w(1),     0];
end