function [q, dq, ddq] = ik_TEOPRUEBA(q0, traj, d_traj, dd_traj, h, parameters, SETTINGS_TEO, current_field)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Inverse Kinematics Algorith - Unit Quaternion %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Author: P.Pierro
% Version: Daniel J. Garc�a-Cano Locatelli
% Version 2: Domingo Esteban

%%% INPUT DATA %%%
% q0: Configuraci�n articular en el tiempo inicial
% traj: Trayectoria de la posici�n y orientaci�n. Es de tipo trayectoria
% d_traj: velocidades de la trayectoria 
% dd_traj: aceleraciones de la trayectoria
% h: Contienes las funciones de la librer�a cinem�tica creada

%%% OUTPUT DATA %%%
% q: espacio articular 
% dq: espacio de las velocidades articulares
% ddq: espacio de las aceleraciones articulares

if nargin < 8
  current_field = 0;
end

% convertir trayectorias a delta
global delta_traj delta_d_traj delta_dd_traj trajo d_trajo dd_trajo
trajo = traj;
d_trajo = d_traj;
dd_trajo = dd_traj;
delta_traj = traj;
delta_d_traj = d_traj;
delta_dd_traj = dd_traj;

for jj=1:(length(SETTINGS_TEO.humanoid_fields)-1)
    delta_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1)=zeros((SETTINGS_TEO.humanoid_fields(jj).size), 1);
    delta_d_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1)=zeros((SETTINGS_TEO.humanoid_fields(jj).size), 1);
    delta_dd_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1)=zeros((SETTINGS_TEO.humanoid_fields(jj).size), 1);
    for kk=2:length(traj.time)
        delta_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk)=traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk)-traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1);%+delta_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk-1);
%         delta_d_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk)=d_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk);
%         delta_dd_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk)=dd_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk);
    end
end


L =length(traj.time);
T = traj.time(L);
q  = zeros(size(q0,1), L);
dq = zeros(size(q0,1), L);
ddq = zeros(size(q0,1), L);
Ts = traj.Ts;
e_p_RF = zeros(3, L);
e_p_LF = zeros(3, L);
e_p_RH = zeros(3, L);
e_p_LH = zeros(3, L);

e_o_RF = zeros(3, L);
e_o_LF = zeros(3, L);
e_o_RH = zeros(3, L);
e_o_LH = zeros(3, L);


% IK parameters
% K_p = parameters.kp;
% K_o = parameters.ko;
% Kp = parameters.kp*eye(3);
% Ko = parameters.ko*eye(3);
Kp = 5*eye(3);
Ko = 5*eye(3);

% Kp = [K_p*eye(3), zeros(3);
%       zeros(3),   K_o*eye(3)]/Ts^2;
% Kd = [K_p*eye(3), zeros(3);
%       zeros(3),   K_o*eye(3)]/Ts;

 
% Initial positions
RF_p0_w = pose_quat2rpy(h.RF_T_w(q0));
LF_p0_w = pose_quat2rpy(h.LF_T_w(q0));
w_p0_RF = pose_quat2rpy(h.w_T_RF(q0));
w_p0_LF = pose_quat2rpy(h.w_T_LF(q0));
CoM_p0_LH = pose_quat2rpy(h.CoM_T_LH(q0));
CoM_p0_RH = pose_quat2rpy(h.CoM_T_RH(q0));
q(:,1) = q0;

    for jj=1:L-1
        %%% Inverse kinematics for Legs %%%
        % Se busca hallar la aceleraci�n articular (ddq). Se necesita:
        % - error de la localizaci�n deseada frente a la localizaci�n actual en
        % cada iteraci�n e=xd-xr
        %       - error del pie derecho (e_RF) y error del pie izquierdo (e_LF)
        %       - error del brazo derecho (e_RH) y error del brazo izquierdo (e_LH)
        % - T�rminos u_R, u_L para las piernas
        % - T�rminos u_RH y u_LH para los brazos

        % Funciones Internas:
        % - invert_kinematics_standard
        % - integrate_vector
%         if strcmp(current_field,'CoM')
            switch traj.SF(jj)
               case 0      % Double Support
               [e_p_RF(:,jj) e_o_RF(:,jj)] = determine_error (delta_traj.CoM(:,jj)+RF_p0_w,...
                   pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));
               [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (delta_traj.CoM(:,jj)+LF_p0_w,...
                   pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));

                    u_R = [Kp*e_p_RF(:,jj); Ko*e_o_RF(:,jj)];
                    u_L = [Kp*e_p_LF(:,jj); Ko*e_o_LF(:,jj)];
%                u_R = [delta_dd_traj.CoM(1:3,jj);...
%                    zeros(3,1)] + Kd * [delta_d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
%                u_L = [delta_dd_traj.CoM(1:3,jj);...
%                    zeros(3,1)] + Kd * [delta_d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);


               dq(1:6, jj) = invert_kinematics_standard (q(:,jj),...
                   h.RF_J_w, u_R, 1:6, 1:6);
               dq(7:12, jj) = invert_kinematics_standard (q(:,jj),...
                   h.LF_J_w, u_L, 1:6, 1:6);          

               case -1     % Right Foot Support

            %             if traj.SF(jj-1)==0
            %                 w_p0_LF = pose_quat2rpy(h.w_T_LF(q(:,jj-1)));
            %             end
               [e_p_RF(:,jj) e_o_RF(:,jj)] = determine_error (delta_traj.CoM(:,jj)+RF_p0_w,...
                   pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));
               [e_p_LF(:,jj) e_o_LF(:,jj)] = determine_error (delta_traj.LF(:,jj)+w_p0_LF,...
                   pose_quat2rpy(real(h.w_T_LF(q(:,jj)))));

                    u_L = [Kp*e_p_LF(:,jj); Ko*e_o_LF(:,jj)];
                    u_R = [Kp*e_p_RF(:,jj); Ko*e_o_RF(:,jj)];
%                u_R = [delta_dd_traj.CoM(1:3,jj);...
%                    zeros(3,1)] + Kd * [delta_d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
%                u_L = [delta_dd_traj.LF(1:3,jj);...
%                    zeros(3,1)] + Kd * [delta_d_traj.LF(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);

               dq(1:6, jj) = invert_kinematics_standard (q(:,jj),...
                   h.RF_J_w, u_R, 1:6, 1:6);
               dq(7:12, jj) = invert_kinematics_standard (q(:,jj),...
                   h.w_J_LF, u_L, 1:6, 1:6);          

               case 1      % Left Foot Support

            %             if traj.SF(jj-1)==0
            %                 w_p0_RF = pose_quat2rpy(h.w_T_RF(q(:,jj-1)));
            %             end
               [e_p_LF(:,jj) e_o_LF(:,jj)]= determine_error (delta_traj.CoM(:,jj)+LF_p0_w,...
                   pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));
               [e_p_RF(:,jj) e_o_RF(:,jj)]= determine_error (delta_traj.RF(:,jj)+w_p0_RF,...
                   pose_quat2rpy(real(h.w_T_RF(q(:,jj)))));


                    u_L = [Kp*e_p_LF(:,jj); Ko*e_o_LF(:,jj)];
                    u_R = [Kp*e_p_RF(:,jj); Ko*e_o_RF(:,jj)];
%                u_L = [delta_dd_traj.CoM(1:3,jj);...
%                    zeros(3,1)] + Kd * [delta_d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);
%                u_R = [delta_dd_traj.RF(1:3,jj);...
%                    zeros(3,1)] + Kd * [delta_d_traj.RF(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);

%                inv(evaluate_jacobian_left(q(:,kk)))*[xd_dot(:,kk) + Kp*error_p(:,kk); wd(:,kk) + Ko*error_o(:,kk)];
               
               dq(7:12, jj) = invert_kinematics_standard (q(:,jj),...
                   h.LF_J_w, u_L, 1:6, 1:6);
               dq(1:6, jj) = invert_kinematics_standard (q(:,jj),...
                   h.w_J_RF, u_R, 1:6, 1:6);  

            end

%             % Waist
%             e_Waist(:,jj) = determine_error (traj.CoM(:,jj),...
%                 pose_quat2rpy(h.CoM_T_RH(q(:,jj))));

            % Right and left arm
            [e_p_RH(:,jj) e_o_RH(:,jj)] = determine_error (delta_traj.RH(:,jj)+CoM_p0_RH,...
                pose_quat2rpy(h.CoM_T_RH(q(:,jj))));
            [e_p_LH(:,jj) e_o_LH(:,jj)] = determine_error (delta_traj.LH(:,jj)+CoM_p0_LH,...
                pose_quat2rpy(h.CoM_T_LH(q(:,jj))));

%             u_RH = delta_dd_traj.RH(:,jj) + Kd * delta_d_traj.RH(:,jj) + Kp * e_RH(:,jj);
%             u_LH = delta_dd_traj.LH(:,jj) + Kd * delta_d_traj.LH(:,jj) + Kp * e_LH(:,jj);
            u_LH = [Kp*e_p_LH(:,jj); Ko*e_o_LH(:,jj)];
            u_RH = [Kp*e_p_RH(:,jj); Ko*e_o_RH(:,jj)];   

            dq(15:20, jj) = invert_kinematics_standard (q(:,jj),...
                h.CoM_J_RH, u_RH, 1:6, 1:6);
            dq(21:26, jj) = invert_kinematics_standard (q(:,jj),...
                h.CoM_J_LH, u_LH, 1:6, 1:6);  
%             dq(:, jj+1)  = integrate_vector (dq(:, jj), ddq(:, jj), Ts);

            q(:, jj+1)   = integrate_vector (q(:, jj), dq(:, jj), Ts);
    end
end

function ddq = invert_kinematics_standard (q_act, J, e, m, n)
J1 = J(q_act);
ddq = J1(m, n)\e(m);
end

function x_next = integrate_vector (x, dx, Ts)
x_next = x + dx * Ts;
end

function [error_p error_o] = determine_error (pd, p)
deseado=pose_rpy2quat(pd);
real=pose_rpy2quat(p);
eta=real(4);
eps=real(5:end);
eta_d=deseado(4);
eps_d=deseado(5:end);

error_p= deseado(1:3) - real(1:3);
error_o = eta*eps_d - eta_d*eps - matrix_S(eps_d)*eps;
end

function S = matrix_S (w)
S = [       0,  -w(3),   w(2);
         w(3),      0,  -w(1);
        -w(2),   w(1),     0];
end