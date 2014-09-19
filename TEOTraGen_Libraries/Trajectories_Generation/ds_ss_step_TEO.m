function [q, dq, ddq, trajectory, d_trajectory, dd_trajectory] = ds_ss_step_TEO(delta, data, support_foot, trajectory, d_trajectory, dd_trajectory)
                      
%DS_SS_STEP_TEO Double Support Step for the robot TEO
%   [Q, DQ, DDQ, TRAJECTORY, D_TRAJECTORY, DD_TRAJECTORY] =
%   DS_SS_STEP_TEP(DELTA, DATA, LEG)
%   returns the joint trajectory, joint velocity, joint acceleration,
%   operational space trajectory, operational space velocity and 
%   acceleration for a double support and simple support step.
%
%   INPUT:
%       DELTA = 
%       DATA = 
%       LEG = 
%
%   See also MOVE_DOUBLE_SUPPORT, MOVE_SIMPLE_SUPPORT.

%   Author: Domingo Esteban
%   References from: P. Pierro and D. Garc�a-Cano
%   RoboticsLab - Universidad Carlos III de Madrid
%   $Revision: 1.0 $  $Date: 2013/07/28 $
% *************************************************************************

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%% DS_SS_STEP_TEO FUNCTION %%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global h TEO pvia 

% INPUTS
    % Steps Data
    L = data.L;     % Step Length
    H = data.H;     % Step Height
    q0 = data.q;   % Initial pose
    
    % Time Parameters
    Ts = data.Ts;   % Sampling time
    t0 = data.t0;   % Initial time
    T = data.T;     % Total time. Time of the step

    % Variation of CoM and FF in Simple Support Phase
    delta_ss_com = [delta.delta_CoM_SS1 delta.delta_CoM_SS2];
    delta_ss_ff = [delta.delta_FF_SS1 delta.delta_FF_SS2];

    % Variation of Right Hand and Left Hand in Simple Support Phase
    delta_RH = delta.delta_RH;
    delta_LH = delta.delta_LH;

    
    humanoid_fields = humanoid_operational_fields(); 
    
    
    % DOMINGO: Ya no va a haber Simple Support, Sólo Double and Simple
    
% switch data.DS_or_SS
%     case 'Double and Simple' % Double Support followed by Simple Support movement
      % Calculate the time of step's phases
      alpha_ds = data.alpha_ds;   % Percentage of time for double support
      alpha_ss = (1-alpha_ds);   % Percentage of time for simple support
      Tinit = trajectory.time(end);
      TDS1 = round_to_Ts(Tinit + alpha_ds*T/2, Ts); % Time for ending of Double Support Phase1
      TSS = round_to_Ts(TDS1 + alpha_ss*T/2, Ts);   % Time for ending of Climbing Simple Phase 1
      TDS2 = round_to_Ts(TSS + alpha_ss*T/2, Ts);  % Time for ending of Landing Simple Phase 1
      Tend = round_to_Ts(TDS2 + alpha_ds*T/2, Ts); % Time for ending of Double Support Phase2

      % Variation of CoM in Double Support phase 1
      delta_ds_com1 = delta.delta_CoM_DS1;
      % Variation of CoM in Double Support phase 2
      delta_ds_com2 = delta.delta_CoM_DS2;

      % Insert Initial operational space positions in the trajectory
      trajectory = insert_trajectory(trajectory, humanoid_fields, create_trajectory_structure(zeros(6,1), Ts, Tinit), 'RF');%(pose_quat2rpy(h.CoM_T_RF(q0)), Ts, Tinit), 'RF');
      trajectory = insert_trajectory(trajectory, humanoid_fields, create_trajectory_structure(zeros(6,1), Ts, Tinit), 'LF');%(pose_quat2rpy(h.CoM_T_LF(q0)), Ts, Tinit), 'LF');
      trajectory = insert_trajectory(trajectory, humanoid_fields, create_trajectory_structure(zeros(6,1), Ts, Tinit), 'RH');%(pose_quat2rpy(h.CoM_T_RH(q0)), Ts, Tinit), 'RH');
      trajectory = insert_trajectory(trajectory, humanoid_fields, create_trajectory_structure(zeros(6,1), Ts, Tinit), 'LH');%(pose_quat2rpy(h.CoM_T_LH(q0)), Ts, Tinit), 'LH');

      % Generate trajectory for the first double support phase
      [trajectory, d_trajectory, dd_trajectory] = move_double_support (delta_ds_com1, Ts, [Tinit; TDS1], trajectory, d_trajectory, dd_trajectory,delta.interpola_CoM_DS1);

      % Generate trajectory for the simple support phase
      [trajectory, d_trajectory, dd_trajectory] = move_simple_support (delta_ss_com, delta_ss_ff, Ts, [TDS1; TSS; TDS2], trajectory, d_trajectory, dd_trajectory, support_foot, delta.interpola_CoM_SS, delta.interpola_FF_SS); % Support on Right foot

      % Generate trajectory for the first double support phase
      [trajectory, d_trajectory, dd_trajectory] = move_double_support (delta_ds_com2, Ts, [TDS2; Tend], trajectory, d_trajectory, dd_trajectory,delta.interpola_CoM_DS1);

      % Generate trajectory for the arms
      [trajectory, d_trajectory, dd_trajectory] = move_arm_simple_support ('RH', delta_RH, Ts, [Tinit; TDS1; TSS; TDS2; Tend], trajectory, d_trajectory, dd_trajectory, delta.interpola_RH);
      [trajectory, d_trajectory, dd_trajectory] = move_arm_simple_support ('LH', delta_LH, Ts, [Tinit; TDS1; TSS; TDS2; Tend], trajectory, d_trajectory, dd_trajectory, delta.interpola_RH);
      
      % Inverse Differential Kinematics Algorithm with left foot
      [q, dq, ddq] = inverse_ds_ss_jacobian_quat(q0, trajectory, d_trajectory, h);
      %[q, dq, ddq] = inverse_ds_ss_jacobian_quat_second_order(q0, trajectory, d_trajectory, dd_trajectory, h);
        
%     case 'Simple' % Only Simple Support movement
%         
%         alpha_ds = 0;
%         TDS1 = 0;
%         TSS = T;
% 
%         delta_ds_com1 = zeros(6,1);
%         
% %         switch support_foot % In Simple Support
% % 
% %           case 'Right Leg' % Right Leg
% % 
%             % Initial positions
%             trajectory = insert_trajectory(trajectory, humanoid_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_RF(q0)), Ts, Tinit), 'RF');
%             trajectory = insert_trajectory(trajectory, humanoid_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_LF(q0)), Ts, Tinit), 'LF');
%             trajectory = insert_trajectory(trajectory, humanoid_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_RH(q0)), Ts, Tinit), 'RH');
%             trajectory = insert_trajectory(trajectory, humanoid_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_LH(q0)), Ts, Tinit), 'LH');
% 
%             [trajectory, d_trajectory, dd_trajectory] = move_simple_support (delta_ss_com, delta_ss_ff, Ts, [TDS1;TDS1+TSS/2;T], trajectory, d_trajectory, dd_trajectory, support_foot,delta.interpola_CoM_SS,delta.interpola_FF_SS); % Support on Right foot
% %             [trajectory, d_trajectory, dd_trajectory] = moving_arm ('RH',delta_RH,zeros(6,1),zeros(6,1),Ts, [Tinit T], trajectory, d_trajectory, dd_trajectory,delta.interpola_RH);
% %             [trajectory, d_trajectory, dd_trajectory] = moving_arm ('LH',delta_LH,zeros(6,1),zeros(6,1),Ts, [Tinit T], trajectory, d_trajectory, dd_trajectory,delta.interpola_LH);
% 
% %             [q, dq, ddq] = inverse_right_ss_TEO(q0, trajectory, d_trajectory, dd_trajectory, h);
% % 
% %           case 'Left Leg' % Left Leg
% % 
% %             % Initial positions
% % 
% %             trajectory = insert_trajectory(trajectory, humanoid_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_LF(q0)), Ts, Tinit), 'LF');% esto pa ver una cosa
% %             trajectory = insert_trajectory(trajectory, humanoid_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_RH(q0)), Ts, Tinit), 'RH');
% %             trajectory = insert_trajectory(trajectory, humanoid_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_LH(q0)), Ts, Tinit), 'LH');
% % 
% %             [trajectory, d_trajectory, dd_trajectory] = move_simple_support (delta_ss_com, delta_ss_ff, Ts, [TDS1;TDS1+TSS/2;T], trajectory, d_trajectory, dd_trajectory,'Simple Support LF',delta.interpola_CoM_SS,delta.interpola_FF_SS); 
% % 
% %             [trajectory, d_trajectory, dd_trajectory] = moving_arm ('RH',delta_RH,zeros(6,1),zeros(6,1),Ts, [Tinit T], trajectory, d_trajectory, dd_trajectory,delta.interpola_RH);
% %             [trajectory, d_trajectory, dd_trajectory] = moving_arm ('LH',delta_LH,zeros(6,1),zeros(6,1),Ts, [Tinit T], trajectory, d_trajectory, dd_trajectory,delta.interpola_LH);
% % 
% %             [q, dq, ddq] = inverse_left_ss_TEO(q0, trajectory, d_trajectory, dd_trajectory, h);
% % 
% % 
% %         end
%         
%         [q, dq, ddq] = inverse_ss_jacobian_quat(q0, trajectory, d_trajectory, h);
%         
% end

