function [q, dq, ddq] = ik_TEO2(q0, traj, d_traj, dd_traj, h, parameters, SETTINGS_TEO, current_field)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Inverse Kinematics Algorith - Unit Quaternion %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Author: P.Pierro
% Version: Daniel J. Garcia-Cano Locatelli
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
trajo=traj;
d_trajo=d_traj;
dd_trajo=dd_traj;
delta_traj=traj;
delta_d_traj=d_traj;
delta_dd_traj=dd_traj;

for jj=1:(length(SETTINGS_TEO.humanoid_fields)-1)
      
%     switch SETTINGS_TEO.humanoid_fields(jj).name
%         case 'CoM'
%             if traj.SF(1)==1
%                 delta_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1)=pose_quat2rpy(h.LF_T_CoM(q0));    
%             else
%                 delta_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1)=pose_quat2rpy(h.RF_T_CoM(q0));
%             end
%         case 'RF'
%             delta_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1)=pose_quat2rpy(h.CoM_T_RF(q0));
%         case 'LF'
%             delta_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1)=pose_quat2rpy(h.CoM_T_LF(q0));
%             pose_quat2rpy(h.CoM_T_LF(q0))
%         case 'RH'
%             delta_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1)=pose_quat2rpy(h.CoM_T_RH(q0));
%         case 'LH'
%             delta_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1)=pose_quat2rpy(h.CoM_T_LH(q0));
%     end
    delta_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1)=zeros((SETTINGS_TEO.humanoid_fields(jj).size), 1);
    delta_d_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1)=zeros((SETTINGS_TEO.humanoid_fields(jj).size), 1);
    delta_dd_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1)=zeros((SETTINGS_TEO.humanoid_fields(jj).size), 1);
    for kk=2:length(traj.time)
        delta_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk)=traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk)-traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1);%+delta_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk-1);
        delta_d_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk)=d_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk)-d_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1);%+delta_d_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk-1);
        delta_dd_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk)=dd_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk)-dd_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,1);%+delta_dd_traj.(SETTINGS_TEO.humanoid_fields(jj).name)(:,kk-1);
    end
end


L =length(traj.time);
T = traj.time(L);
q  = zeros(size(q0,1), L);
dq = zeros(size(q0,1), L);
ddq = zeros(size(q0,1), L);
Ts = traj.Ts;
e_RF = zeros(6, L);
e_LF = zeros(6, L);
e_RH = zeros(6, L);
e_LH = zeros(6, L);
e_humanoid_part = zeros(6, L);

% IK parameters
K_p = parameters.kp;
K_o = parameters.ko;

Kp = [K_p*eye(3), zeros(3);
      zeros(3),   K_o*eye(3)]/Ts^2;
Kd = [K_p*eye(3), zeros(3);
      zeros(3),   K_o*eye(3)]/Ts;

 
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
               e_RF(:,jj) = evaluate_error (delta_traj.CoM(:,jj)+RF_p0_w,...
                   pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));
               e_LF(:,jj) = evaluate_error (delta_traj.CoM(:,jj)+LF_p0_w,...
                   pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));


               u_R = [delta_dd_traj.CoM(1:3,jj);...
                   zeros(3,1)] + Kd * [delta_d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
               u_L = [delta_dd_traj.CoM(1:3,jj);...
                   zeros(3,1)] + Kd * [delta_d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);


               ddq(1:6, jj+1) = invert_kinematics_standard (q(:,jj),...
                   h.RF_J_w, u_R, 1:6, 1:6);
               ddq(7:12, jj+1) = invert_kinematics_standard (q(:,jj),...
                   h.LF_J_w, u_L, 1:6, 1:6);          

               case -1     % Right Foot Support

            %             if traj.SF(jj-1)==0
            %                 w_p0_LF = pose_quat2rpy(h.w_T_LF(q(:,jj-1)));
            %             end
               e_RF(:,jj) = evaluate_error (delta_traj.CoM(:,jj)+RF_p0_w,...
                   pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));
               e_LF(:,jj) = evaluate_error (delta_traj.LF(:,jj)+w_p0_LF,...
                   pose_quat2rpy(real(h.w_T_LF(q(:,jj)))));


               u_R = [delta_dd_traj.CoM(1:3,jj);...
                   zeros(3,1)] + Kd * [delta_d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
               u_L = [delta_dd_traj.LF(1:3,jj);...
                   zeros(3,1)] + Kd * [delta_d_traj.LF(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);

               ddq(1:6, jj+1) = invert_kinematics_standard (q(:,jj),...
                   h.RF_J_w, u_R, 1:6, 1:6);
               ddq(7:12, jj+1) = invert_kinematics_standard (q(:,jj),...
                   h.w_J_LF, u_L, 1:6, 1:6);          

               case 1      % Left Foot Support

            %             if traj.SF(jj-1)==0
            %                 w_p0_RF = pose_quat2rpy(h.w_T_RF(q(:,jj-1)));
            %             end
               e_LF(:,jj) = evaluate_error (delta_traj.CoM(:,jj)+LF_p0_w,...
                   pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));
               e_RF(:,jj) = evaluate_error (delta_traj.RF(:,jj)+w_p0_RF,...
                   pose_quat2rpy(real(h.w_T_RF(q(:,jj)))));


               u_L = [delta_dd_traj.CoM(1:3,jj);...
                   zeros(3,1)] + Kd * [delta_d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);
               u_R = [delta_dd_traj.RF(1:3,jj);...
                   zeros(3,1)] + Kd * [delta_d_traj.RF(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);

               ddq(7:12, jj+1) = invert_kinematics_standard (q(:,jj),...
                   h.LF_J_w, u_L, 1:6, 1:6);
               ddq(1:6, jj+1) = invert_kinematics_standard (q(:,jj),...
                   h.w_J_RF, u_R, 1:6, 1:6);  

            end

%             % Waist
%             e_Waist(:,jj) = evaluate_error (traj.CoM(:,jj),...
%                 pose_quat2rpy(h.CoM_T_RH(q(:,jj))));

            % Right and left arm
            e_RH(:,jj) = evaluate_error (delta_traj.RH(:,jj)+CoM_p0_RH,...
                pose_quat2rpy(h.CoM_T_RH(q(:,jj))));
            e_LH(:,jj) = evaluate_error (delta_traj.LH(:,jj)+CoM_p0_LH,...
                pose_quat2rpy(h.CoM_T_LH(q(:,jj))));

            u_RH = delta_dd_traj.RH(:,jj) + Kd * delta_d_traj.RH(:,jj) + Kp * e_RH(:,jj);
            u_LH = delta_dd_traj.LH(:,jj) + Kd * delta_d_traj.LH(:,jj) + Kp * e_LH(:,jj);

            ddq(15:20, jj+1) = invert_kinematics_standard (q(:,jj),...
                h.CoM_J_RH, u_RH, 1:6, 1:6);
            ddq(21:26, jj+1) = invert_kinematics_standard (q(:,jj),...
                h.CoM_J_LH, u_LH, 1:6, 1:6);          

            dq(:, jj+1)  = integrate_vector (dq(:, jj), ddq(:, jj+1), Ts);
            q(:, jj+1)   = integrate_vector (q(:, jj), dq(:, jj+1), Ts);
% 
%         elseif strcmp(current_field,'LF')
%             switch traj.SF(jj)
%                case 0      % Double Support
%                e_RF(:,jj) = evaluate_error (traj.CoM(:,jj)+RF_p0_w,...
%                    pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));
%                e_LF(:,jj) = evaluate_error (traj.CoM(:,jj)+LF_p0_w,...
%                    pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));
% 
% 
%                u_R = [dd_traj.CoM(1:3,jj);...
%                    zeros(3,1)] + Kd * [d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
%                u_L = [dd_traj.CoM(1:3,jj);...
%                    zeros(3,1)] + Kd * [d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);
% 
% 
%                ddq(1:6, jj+1) = invert_kinematics_standard (q(:,jj),...
%                    h.RF_J_w, u_R, 1:6, 1:6);
%                ddq(7:12, jj+1) = invert_kinematics_standard (q(:,jj),...
%                    h.LF_J_w, u_L, 1:6, 1:6);          
% 
%                case -1     % Right Foot Support
% 
%             %             if traj.SF(jj-1)==0
%             %                 w_p0_LF = pose_quat2rpy(h.w_T_LF(q(:,jj-1)));
%             %             end
%                e_RF(:,jj) = evaluate_error (traj.CoM(:,jj)+RF_p0_w,...
%                    pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));
%                e_LF(:,jj) = evaluate_error (traj.LF(:,jj)+w_p0_LF,...
%                    pose_quat2rpy(real(h.w_T_LF(q(:,jj)))));
% 
% 
%                u_R = [dd_traj.CoM(1:3,jj);...
%                    zeros(3,1)] + Kd * [d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
%                u_L = [dd_traj.LF(1:3,jj);...
%                    zeros(3,1)] + Kd * [d_traj.LF(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);
% 
%                ddq(1:6, jj+1) = invert_kinematics_standard (q(:,jj),...
%                    h.RF_J_w, u_R, 1:6, 1:6);
%                ddq(7:12, jj+1) = invert_kinematics_standard (q(:,jj),...
%                    h.w_J_LF, u_L, 1:6, 1:6);          
% 
%                case 1      % Left Foot Support
% 
%             %             if traj.SF(jj-1)==0
%             %                 w_p0_RF = pose_quat2rpy(h.w_T_RF(q(:,jj-1)));
%             %             end
%                e_LF(:,jj) = evaluate_error (traj.CoM(:,jj)+LF_p0_w,...
%                    pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));
%                e_RF(:,jj) = evaluate_error (traj.RF(:,jj)+w_p0_RF,...
%                    pose_quat2rpy(real(h.w_T_RF(q(:,jj)))));
% 
% 
%                u_L = [dd_traj.CoM(1:3,jj);...
%                    zeros(3,1)] + Kd * [d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);
%                u_R = [dd_traj.RF(1:3,jj);...
%                    zeros(3,1)] + Kd * [d_traj.RF(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
% 
%                ddq(7:12, jj+1) = invert_kinematics_standard (q(:,jj),...
%                    h.LF_J_w, u_L, 1:6, 1:6);
%                ddq(1:6, jj+1) = invert_kinematics_standard (q(:,jj),...
%                    h.w_J_RF, u_R, 1:6, 1:6);  
% 
%             end
% 
%             % Waist
%             e_Waist(:,jj) = evaluate_error (traj.CoM(:,jj),...
%                 pose_quat2rpy(h.CoM_T_RH(q(:,jj))));
% 
%             % Right and left arm
%             e_RH(:,jj) = evaluate_error (traj.RH(:,jj),...
%                 pose_quat2rpy(h.CoM_T_RH(q(:,jj))));
%             e_LH(:,jj) = evaluate_error (traj.LH(:,jj),...
%                 pose_quat2rpy(h.CoM_T_LH(q(:,jj))));
% 
%             u_RH = dd_traj.RH(:,jj) + Kd * d_traj.RH(:,jj) + Kp * e_RH(:,jj);
%             u_LH = dd_traj.LH(:,jj) + Kd * d_traj.LH(:,jj) + Kp * e_LH(:,jj);
% 
%             ddq(14:18, jj+1) = invert_kinematics_standard (q(:,jj),...
%                 h.CoM_J_RH, u_RH, 1:3, 1:5);
%             ddq(19:23, jj+1) = invert_kinematics_standard (q(:,jj),...
%                 h.CoM_J_LH, u_LH, 1:3, 1:5);          
% 
%             dq(:, jj+1)  = integrate_vector (dq(:, jj), ddq(:, jj+1), Ts);
%             q(:, jj+1)   = integrate_vector (q(:, jj), dq(:, jj+1), Ts);
%             
%         else
%             % TEO is standing with both foot on the floor
%             pose_RF = [0; -SETTINGS_TEO.TEO.legs.link_lengths(1)+SETTINGS_TEO.TEO.legs.link_lengths(4); 0; 0; 0; 0];
%             pose_LF = [0; SETTINGS_TEO.TEO.legs.link_lengths(1)-SETTINGS_TEO.TEO.legs.link_lengths(4); 0; 0; 0; 0];
%             
%             switch current_field
%                 case 'CoM'
% %                     if strcmp(SETTINGS_TEO.units.pos,'m') && strcmp(SETTINGS_TEO.units.orient,'rad')
% %                         real_pose_CoM = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_RF))*pose_quat2tr(SETTINGS_TEO.h.RF_T_CoM(q(:,jj))));  % RF is the standing leg
% %                     end
% %                     e_humanoid_part(:,jj) = evaluate_error (traj.CoM(:,jj), real_pose_CoM);
% %                     u_humanoid_part = [dd_traj.CoM(1:3,jj); zeros(3,1)] + Kd * [d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_humanoid_part(:,jj);
% %                     dq=0;
% %                     ddq=0;
% %                         real_pose_w = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_RF))*pose_quat2tr(SETTINGS_TEO.h.RF_T_w(q(:,jj))));
% %                         real_pose_CoM = pose_tr2rpy(pose_quat2tr(SETTINGS_TEO.h.w_T_CoM(q(:,jj)))*pose_quat2tr(pose_rpy2quat(pose_RF))*pose_quat2tr(SETTINGS_TEO.h.RF_T_w(q(:,jj))));
% %                         desired_pose_CoM = traj.CoM(:,jj);
% %                         desired_pose_Waist = %Asumo que quiero que varie igual que
%                e_RF(:,jj) = evaluate_error (delta_traj.CoM(:,jj)+RF_p0_w,...
%                    pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));
%                e_LF(:,jj) = evaluate_error (delta_traj.CoM(:,jj)+LF_p0_w,...
%                    pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));
% 
% 
%                u_R = [delta_dd_traj.CoM(1:3,jj);...
%                    zeros(3,1)] + Kd * [delta_d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
%                u_L = [delta_dd_traj.CoM(1:3,jj);...
%                    zeros(3,1)] + Kd * [delta_d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);
% 
% 
%                ddq(1:6, jj+1) = invert_kinematics_standard (q(:,jj),...
%                    h.RF_J_w, u_R, 1:6, 1:6);
%                ddq(7:12, jj+1) = invert_kinematics_standard (q(:,jj),...
%                    h.LF_J_w, u_L, 1:6, 1:6);   
%                
%                 case 'RF'
%                    e_LF(:,jj) = evaluate_error (delta_traj.CoM(:,jj)+LF_p0_w,...
%                        pose_quat2rpy(real(h.LF_T_w(q(:,jj)))));
%                    e_RF(:,jj) = evaluate_error (delta_traj.RF(:,jj)+w_p0_RF,...
%                        pose_quat2rpy(real(h.w_T_RF(q(:,jj)))));
% 
% 
%                    u_L = [delta_dd_traj.CoM(1:3,jj);...
%                        zeros(3,1)] + Kd * [delta_d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);
%                    u_R = [dd_traj.RF(1:3,jj);...
%                        zeros(3,1)] + Kd * [delta_d_traj.RF(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
% 
%                    ddq(7:12, jj+1) = invert_kinematics_standard (q(:,jj),...
%                        h.LF_J_w, u_L, 1:6, 1:6);
%                    ddq(1:6, jj+1) = invert_kinematics_standard (q(:,jj),...
%                        h.w_J_RF, u_R, 1:6, 1:6);  
%                
%                 case 'LF'
%                     %Asumimos que solo se mueve la pierna y todo lo demas
%                     %no se mueve
%                    e_RF(:,jj) = evaluate_error (delta_traj.CoM(:,jj)+RF_p0_w,...
%                    pose_quat2rpy(real(h.RF_T_w(q(:,jj)))));
%                    e_LF(:,jj) = evaluate_error (delta_traj.LF(:,jj)+w_p0_LF,...
%                        pose_quat2rpy(real(h.w_T_LF(q(:,jj)))));
% 
% 
%                    u_R = [delta_dd_traj.CoM(1:3,jj);...
%                        zeros(3,1)] + Kd * [delta_d_traj.CoM(1:3,jj);zeros(3,1)] + Kp*e_RF(:,jj);
%                    u_L = [delta_dd_traj.LF(1:3,jj);...
%                        zeros(3,1)] + Kd * [delta_d_traj.LF(1:3,jj);zeros(3,1)] + Kp*e_LF(:,jj);
% 
%                    ddq(1:6, jj+1) = invert_kinematics_standard (q(:,jj),...
%                        h.RF_J_w, u_R, 1:6, 1:6);
%                    ddq(7:12, jj+1) = invert_kinematics_standard (q(:,jj),...
%                        h.w_J_LF, u_L, 1:6, 1:6);    
%                 case 'RH'
%                    e_RH(:,jj) = evaluate_error (delta_traj.RH(:,jj)+CoM_p0_RH,...
%                         pose_quat2rpy(h.CoM_T_RH(q(:,jj))));
% 
%                    u_RH = delta_dd_traj.RH(:,jj) + Kd * delta_d_traj.RH(:,jj) + Kp * e_RH(:,jj);
%             
%                    ddq(15:20, jj+1) = invert_kinematics_standard (q(:,jj),...
%                         h.CoM_J_RH, u_RH, 1:3, 1:6);
%         
%                 case 'LH'
%                 	e_LH(:,jj) = evaluate_error (delta_traj.LH(:,jj)+CoM_p0_LH,...
%                     pose_quat2rpy(h.CoM_T_LH(q(:,jj))));
%                 
%                     u_LH = delta_dd_traj.LH(:,jj) + Kd * delta_d_traj.LH(:,jj) + Kp * e_LH(:,jj);
%                     
%                     ddq(21:26, jj+1) = invert_kinematics_standard (q(:,jj),...
%                         h.CoM_J_LH, u_LH, 1:3, 1:6);  
%                     
%             end
%             dq(:, jj+1)  = integrate_vector (dq(:, jj), ddq(:, jj+1), Ts);
%             q(:, jj+1)   = integrate_vector (q(:, jj), dq(:, jj+1), Ts);
%         end

    end
end

function ddq = invert_kinematics_standard (q_act, J, e, m, n)
J1 = J(q_act);
ddq = J1(m, n)\e(m);
end

function x_next = integrate_vector (x, dx, Ts)
x_next = x + dx * Ts;
end