clear all
clc

disp('TEO Humanoid Robot - Trajectory Generation')
disp('Robotics Lab, Universidad Carlos III de Madrid.')
disp('<a href = "http://roboticslab.uc3m.es">http://roboticslab.uc3m.es</a>')
disp(' ')
disp('Please wait some seconds...')
disp(' ')
% Generate TEO structure and library
TEO = TEO_structure('numeric', 'rad', 'm');
h = TEO_kinematics_library;



% Steps Data
%global data
data.TS = 0.02;                                 % Sampling Time
data.t0 = 0;                                    % Initial time
data.T = 4;                                     % Total time. Time of the step
data.alpha_ds = 1/3;                            % Percentage of the total time for double support
data.alpha_sf = 0.2;                            % Percentage of the total time for support foot ???
data.DS_or_SS = 'Double and Simple';            %
data.L = 0.05;              % Length of the step (X direction)
data.H = 0.03;              % Height of the step (Z direction)
data.q0 = [0; 0.00325683448936741; -0.308647699300050; 0.796421295515307; -0.487773596215257; 0.0278918646012491;...                % Right Leg
     0; 0.00325683448936741; -0.311486990906165; 0.796421295515307; -0.484850796032492; -0.0354911450764397;...                     % Left Leg
     0.0349065850398866; 0;...                                                                                                      % Waist
     1.57079632679490; -0.167017153300893; 0; -0.734875474523928; 0; 0;                                                             % Right Arm
     1.57079632679490; 0.167017153300893; 0;  -0.734875474523928; 0; 0];                                                            % Left Arm 
data.Leg='Right Leg'; % Support foot
 
while 1
    % Change step parameters
    disp(' ');
    disp('******************');
    disp('Step Parameters: ');
    disp(['TS = ', num2str(data.TS)]);
    disp(['t0 = ', num2str(data.t0)]);
    disp(['T = ', num2str(data.T)]);
    disp(['DS_or_SS = ', data.DS_or_SS]);
    disp(['alpha_ds = ', num2str(data.alpha_ds)]);
    disp(['alpha_sf = ', num2str(data.alpha_sf)]);
    disp(['Leg = ', data.Leg]);
    
    disp(['L = ', num2str(data.L)]);
    disp(['H = ', num2str(data.H)]);
    
    selection=input('Select the name of the parameter or press q: ','s');
    if selection=='q'
        break
    elseif  isfield(data, selection)
        value=input('Parameter value: ','s');
        if (strcmp(selection,'DS_or_SS') || strcmp(selection,'Leg'))
            data.(selection)=value;
        else
            data.(selection)=str2double(value);
        end
    else
        disp(['Error: ', selection, ' is not an option']);
    end
end

Leg=data.Leg;


% Delta Parameters

% Pre-evaluate delta x and delta y CoM in Double Support
switch Leg
    case 'Right Leg' % Support on right foot
        delta = h.CoM_T_RF(data.q0);
        delta_P(1,:) = (2*delta(1) + (TEO.legs.right.foot.limits.x(1) + TEO.legs.right.foot.limits.x(2))/1);
        delta_P(2,:) = (2*delta(2) + (TEO.legs.right.foot.limits.y(1) + TEO.legs.right.foot.limits.y(2))/1);
        
    case 'Left Leg' % Support on left foot
        delta = h.CoM_T_LF(data.q0);
        delta_P(1,:) = (2*delta(1) + (TEO.legs.left.foot.limits.x(1) + TEO.legs.left.foot.limits.x(2))/1);
        delta_P(2,:) = (2*delta(2) + (TEO.legs.left.foot.limits.y(1) + TEO.legs.left.foot.limits.y(2))/1);
end 

delta_P = delta_P/2; 

X1_SF = data.L*data.alpha_sf/2;
X2_SF = data.L*data.alpha_sf;
X1_FF = data.L*(1-data.alpha_sf)/2;
X2_FF = data.L*(1-data.alpha_sf);
pvia = zeros(3,1);
 
% Delta Data
%global delta 
delta.delta_CoM_DS = [delta_P(1);...        % Variation of the CoM in the Double Support phase
        delta_P(2);...                      
        0;...                               
        zeros(3,1)];                    
delta.interpola_CoM_DS = 'Polynomial5';          % Interpolation for the CoM in the Double Support phase
    
delta.delta_CoM_SS1 = [X1_SF;...            % Variation of the CoM in the Simple Support phase 1
        0;...
        0;...
        zeros(3,1)];

delta.delta_CoM_SS2 = [X2_SF;...            % Variation of the CoM in the Simple Support phase 2
    0;...
    0;...
    zeros(3,1)];
delta.interpola_CoM_SS = 'Polynomial5';          % Interpolation for the CoM in the Simple Support phase (1 and 2)

delta.delta_FF_SS1 = [X1_FF;...             % Variation of the Floating Foot in the Simple Support phase 1  
    0;...
    data.H;...
    zeros(3,1)];

delta.delta_FF_SS2 = [X2_FF;...             % Variation of the Floating Foot in the Simple Support phase 1  
    0;...
    0;...
    zeros(3,1)];
delta.interpola_FF_SS = 'Polynomial5';           % Interpolation for the Floating Foot in the Simple Support phase (1 and 2)

delta.delta_RH = [0;...                     % Variation of the Right Hand in the DS and SS phase
    0;...
    0;...
    zeros(3,1)];
delta.interpola_RH = 'Polynomial5';              % Interpolation for the Right Hand in the DS and SS phase

delta.delta_LH = [0;...                     % Variation of the Left Hand in the DS and SS phase
    0;...
    0;...
    zeros(3,1)];
delta.interpola_LH = 'Polynomial5';              % Interpolation for the Left Hand in the DS and SS phase

while 1
    disp(' ');
    disp('******************');
    disp('Delta Parameters: ');
    disp(['delta_CoM_DS:  deltax=', num2str(delta.delta_CoM_DS(1)), '  deltay=', num2str(delta.delta_CoM_DS(2)),'  deltaz=', num2str(delta.delta_CoM_DS(3))]);
    disp(['interpola_CoM_DS=',delta.interpola_CoM_DS]);
    disp(['delta_CoM_SS1:  deltax=', num2str(delta.delta_CoM_SS1(1)), '  deltay=', num2str(delta.delta_CoM_SS1(2)),'  deltaz=', num2str(delta.delta_CoM_SS1(3))]);
    disp(['delta_CoM_SS2:  deltax=', num2str(delta.delta_CoM_SS2(1)), '  deltay=', num2str(delta.delta_CoM_SS2(2)),'  deltaz=', num2str(delta.delta_CoM_SS2(3))]);
    disp(['interpola_CoM_SS=', delta.interpola_CoM_SS]);
    disp(['delta_FF_SS1:  deltax=', num2str(delta.delta_FF_SS1(1)), '  deltay=', num2str(delta.delta_FF_SS1(2)),'  deltaz=', num2str(delta.delta_FF_SS1(3))]);
    disp(['delta_FF_SS2:  deltax=', num2str(delta.delta_FF_SS2(1)), '  deltay=', num2str(delta.delta_FF_SS2(2)),'  deltaz=', num2str(delta.delta_FF_SS2(3))]);
    disp(['interpola_FF_SS=', num2str(delta.interpola_FF_SS)]);
    disp(['delta_RH:  deltax=', num2str(delta.delta_RH(1)), '  deltay=', num2str(delta.delta_RH(2)),'  deltaz=', num2str(delta.delta_RH(3))]);
    disp(['interpola_RH=',delta.interpola_RH]);
    disp(['delta_LH:  deltax=', num2str(delta.delta_LH(1)), '  deltay=', num2str(delta.delta_LH(2)),'  deltaz=', num2str(delta.delta_LH(3))]);
    disp(['interpola_LH=',delta.interpola_LH]);
    
    
    selection=input('Select the name of the parameter or press q: ','s');
    if selection=='q'
        break
    elseif  isfield(delta, selection)
        if (strcmp(selection,'interpola_CoM_DS') || strcmp(selection,'interpola_CoM_SS') || strcmp(selection,'interpola_FF_SS') || strcmp(selection,'interpola_RH') || strcmp(selection,'interpola_LH'))
            delta.(selection)=input('Parameter value: ','s');
        else
            direction=input('Delta direction ("x", "y" or "z"): ','s');
            if direction=='x'
                value=input('Parameter value: ','s');
                delta.(selection)(1)=str2double(value);
            elseif direction=='y'
                value=input('Parameter value: ','s');
                delta.(selection)(2)=str2double(value);
            elseif direction=='z'
                value=input('Parameter value: ','s');
                delta.(selection)(3)=str2double(value);
            else
                disp(['Error: ', direction, ' is not an option']);
            end
            %data.(selection)=str2double(value);
        end
    else
        disp(['Error: ', selection, ' is not an option']);
    end
end





%global q dq ddq
%global trajectory d_trajectory dd_trajectory
 
%[q,dq,ddq,trajectory,d_trajectory,dd_trajectory] = ds_ss_step_TEO(delta,data,Leg);

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%% DS_SS_STEP_TEO FUNCTION %%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% INPUTS
    % Steps Data
    L = data.L;     % Step Length
    H = data.H;     % Step Height
    q0 = data.q0;   % Initial pose
    
    % Time Parameters
    Ts = data.TS;   % Sampling time
    t0 = data.t0;   % Initial time
    T = data.T;     % Total time. Time of the step

    % Variation of CoM and FF in Simple Support Phase
    delta_ss_com = [delta.delta_CoM_SS1 delta.delta_CoM_SS2];
    delta_ss_ff = [delta.delta_FF_SS1 delta.delta_FF_SS2];

    % Variation of Right Hand and Left Hand in Simple Support Phase
    delta_RH = delta.delta_RH;
    delta_LH = delta.delta_LH;

% CREATE TRAJECTORY
    % Fields corresponding to the operational space of TEO
    TEO_fields = humanoid_operational_fields (); 

    % Creates the trajectories structures
    trajectory = create_trajectory_template (TEO_fields, Ts);
    d_trajectory = create_trajectory_template (TEO_fields, Ts);
    dd_trajectory = create_trajectory_template (TEO_fields, Ts);
    

% TEO INITIAL POSITION
% TEO is standing with both foot on the floor
pose_RF = [0; -TEO.legs.link_lengths(1)+TEO.legs.link_lengths(4); 0; 0; 0; 0];
pose_LF = [0; TEO.legs.link_lengths(1)-TEO.legs.link_lengths(4); 0; 0; 0; 0];
pose_CoM = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_RF))*pose_quat2tr(h.RF_T_CoM(q0)));
pose_RH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_quat2tr(h.CoM_T_RH(q0)));
pose_LH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_quat2tr(h.CoM_T_LH(q0)));
    
switch data.DS_or_SS
    case 'Double and Simple' % Double Support followed by Simple Support movement
        % Calculate the time of step's phases
        alpha_ds = data.alpha_ds;   % Percentage of time for double support
        alpha_sf = data.alpha_sf;   % Percentage of time for SF ????
        T_ds = round_to_Ts(alpha_ds * T, Ts);       % Total time for Double Support
        T_ss = round_to_Ts((1-alpha_ds) * T, Ts);   % Total time for Simple Support

        % Variation of CoM in Double Support phase
        delta_ds_com = delta.delta_CoM_DS;

        switch Leg %Select the support leg for the Simple Support phase
            case 'Right Leg' % Right Leg
                
                % Insert Initial operational space positions in the
                % trajectory
                
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'RF');%(pose_quat2rpy(h.CoM_T_RF(q0)), Ts, t0), 'RF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'LF');%(pose_quat2rpy(h.CoM_T_LF(q0)), Ts, t0), 'LF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'RH');%(pose_quat2rpy(h.CoM_T_RH(q0)), Ts, t0), 'RH');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'LH');%(pose_quat2rpy(h.CoM_T_LH(q0)), Ts, t0), 'LH');

                % Generate trajectory for the double support phase
                [trajectory, d_trajectory, dd_trajectory] = move_double_support2 (delta_ds_com, Ts, [t0;T_ds], trajectory, d_trajectory, dd_trajectory,delta.interpola_CoM_DS);

                % Generate trajectory for the simple support phase
                [trajectory, d_trajectory, dd_trajectory] = move_simple_support (delta_ss_com, delta_ss_ff, Ts, [T_ds;T_ds+T_ss/2;T], trajectory, d_trajectory, dd_trajectory, 'Simple Support RF',delta.interpola_CoM_SS,delta.interpola_FF_SS); % Support on Right foot
                
                % Generate trajectory for the double support phase 2
                delta_ds_com(1) = 0;
                delta_ds_com(2) = -2*delta_ds_com(2);
                
%                 [trajectory, d_trajectory, dd_trajectory] = move_double_support2 (delta_ds_com, Ts, [T;T+T_ds], trajectory, d_trajectory, dd_trajectory,delta.interpola_CoM_DS);
                
%                 % Generate trajectory for the simple support phase 2
%                 delta.delta_CoM_SS1(2)=-delta.delta_CoM_SS1(2);
%                 delta.delta_CoM_SS2(2)=-delta.delta_CoM_SS2(2);
%                 delta_ss_com = [delta.delta_CoM_SS1 delta.delta_CoM_SS2];
%                 delta.delta_FF_SS1(2)=-delta.delta_FF_SS1(2);
%                 delta.delta_FF_SS2(2)=-delta.delta_FF_SS2(2); 
%                 delta.delta_FF_SS1(1) = data.L*(1-data.alpha_sf);
%                 delta.delta_FF_SS2(1) = data.L*data.alpha_sf;
%                 delta_ss_ff = [delta.delta_FF_SS1 delta.delta_FF_SS2];
%                 [trajectory, d_trajectory, dd_trajectory] = move_simple_support (delta_ss_com, delta_ss_ff, Ts, [T+T_ds;T+T_ds+T_ss/2;2*T], trajectory, d_trajectory, dd_trajectory, 'Simple Support RF',delta.interpola_CoM_SS,delta.interpola_FF_SS); % Support on Left foot
                
                % Generate trajectory for the arms
                %[trajectory, d_trajectory, dd_trajectory] = moving_arm ('RH',delta_RH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_RH);
                %[trajectory, d_trajectory, dd_trajectory] = moving_arm ('LH',delta_LH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_LH);

                % Inverse Differential Kinematics Algorithm with right foot
                % as support
                [q, dq, ddq] = inverse_right_ds_ss_TEO(q0, trajectory, d_trajectory, dd_trajectory, h);

            case 'Left Leg' % Left Leg
                
                % Insert Initial operational space positions in the
                % trajectory
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'RF');%(pose_quat2rpy(h.CoM_T_RF(q0)), Ts, t0), 'RF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'LF');%(pose_quat2rpy(h.CoM_T_LF(q0)), Ts, t0), 'LF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'RH');%(pose_quat2rpy(h.CoM_T_RH(q0)), Ts, t0), 'RH');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'LH');%(pose_quat2rpy(h.CoM_T_LH(q0)), Ts, t0), 'LH');

                % Generate trajectory for the double support phase
                [trajectory, d_trajectory, dd_trajectory] = move_double_support (delta_ds_com, Ts, [t0;T_ds], trajectory, d_trajectory, dd_trajectory,delta.interpola_CoM_DS);

                % Generate trajectory for the simple support phase
                [trajectory, d_trajectory, dd_trajectory] = move_simple_support (delta_ss_com, delta_ss_ff, Ts, [T_ds;T_ds+T_ss/2;T], trajectory, d_trajectory, dd_trajectory,'Simple Support LF',delta.interpola_CoM_SS,delta.interpola_FF_SS); 

                % Generate trajectory for the arms
%                 [trajectory, d_trajectory, dd_trajectory] = moving_arm ('RH',delta_RH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_RH);
%                 [trajectory, d_trajectory, dd_trajectory] = moving_arm ('LH',delta_LH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_LH);

                % Inverse Differential Kinematics Algorithm with left foot
                % as support
                [q, dq, ddq] = inverse_left_ds_ss_TEO(q0, trajectory, d_trajectory, dd_trajectory, h);
        end
        
    case 'Simple' % Only Simple Support movement
        
        alpha_ds = 0;
        alpha_sf = data.alpha_sf;
        T_ds = 0;
        T_ss = T;

        delta_ds_com = zeros(6,1);
        
        switch Leg % In Simple Support

            case 'Right Leg' % Right Leg

                % Initial positions
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'RF');%(pose_quat2rpy(h.CoM_T_RF(q0)), Ts, t0), 'RF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, 0), 'LF');%(pose_quat2rpy(h.CoM_T_LF(q0)), Ts, 0), 'LF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'RH');%(pose_quat2rpy(h.CoM_T_RH(q0)), Ts, t0), 'RH');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(zeros(6,1), Ts, t0), 'LH');%(pose_quat2rpy(h.CoM_T_LH(q0)), Ts, t0), 'LH');

                [trajectory, d_trajectory, dd_trajectory] = move_simple_support (delta_ss_com, delta_ss_ff, Ts, [T_ds;T_ds+T_ss/2;T], trajectory, d_trajectory, dd_trajectory, 'Simple Support RF',delta.interpola_CoM_SS,delta.interpola_FF_SS); % Support on Right foot
                [trajectory, d_trajectory, dd_trajectory] = moving_arm ('RH',delta_RH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_RH);
                [trajectory, d_trajectory, dd_trajectory] = moving_arm ('LH',delta_LH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_LH);

                [q, dq, ddq] = inverse_right_ss_TEO(q0, trajectory, d_trajectory, dd_trajectory, h);

            case 'Left Leg' % Left Leg

                % Initial positions
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_RF(q0)), Ts, t0), 'RF');%(pose_quat2rpy(h.CoM_T_RF(q0)), Ts, t0), 'RF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_LF(q0)), Ts, t0), 'LF');%(pose_quat2rpy(h.CoM_T_LF(q0)), Ts, t0), 'LF');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_RH(q0)), Ts, t0), 'RH');%(pose_quat2rpy(h.CoM_T_RH(q0)), Ts, t0), 'RH');
                trajectory = insert_trajectory(trajectory, TEO_fields, create_trajectory_structure(pose_quat2rpy(h.CoM_T_LH(q0)), Ts, t0), 'LH');%(pose_quat2rpy(h.CoM_T_LH(q0)), Ts, t0), 'LH');

                [trajectory, d_trajectory, dd_trajectory] = move_simple_support (delta_ss_com, delta_ss_ff, Ts, [T_ds;T_ds+T_ss/2;T], trajectory, d_trajectory, dd_trajectory,'Simple Support LF',delta.interpola_CoM_SS,delta.interpola_FF_SS); 

                [trajectory, d_trajectory, dd_trajectory] = moving_arm ('RH',delta_RH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_RH);
                [trajectory, d_trajectory, dd_trajectory] = moving_arm ('LH',delta_LH,zeros(6,1),zeros(6,1),Ts, [t0 T], trajectory, d_trajectory, dd_trajectory,delta.interpola_LH);

                [q, dq, ddq] = inverse_left_ss_TEO(q0, trajectory, d_trajectory, dd_trajectory, h);


        end
        
end

%%% Other things
actualR=h.RF_T_CoM(q0);

com_realR = pose_quat2rpy(h.RF_T_CoM(q));

if (size(com_realR,2)~=size(trajectory.CoM,2))
    for j=1:size(trajectory.CoM,2)-1, %-1 porque en simple support "desaparece un valor de q"
        com_deseadoR(:,j) = pose_quat2rpy(actualR)+trajectory.CoM(:,j);
    end
else
    for j=1:size(trajectory.CoM,2), %-1 porque en simple support "desaparece un valor de q"
        com_deseadoR(:,j) = pose_quat2rpy(actualR)+trajectory.CoM(:,j);
    end 
end



% errorR=com_deseadoR-com_realR;
% 
% errorR_prom=mean(errorR,2);
% 
% actualL=h.LF_T_CoM(q0);
% 
% com_realL = pose_quat2rpy(h.LF_T_CoM(q));
% 
% if (size(com_realL,2)~=size(trajectory.CoM,2))
%     for j=1:size(trajectory.CoM,2)-1, %-1 porque en simple support "desaparece un valor de q"
%         com_deseadoL(:,j) = pose_quat2rpy(actualL)+trajectory.CoM(:,j);
%     end
% else
%     for j=1:size(trajectory.CoM,2), %-1 porque en simple support "desaparece un valor de q"
%         com_deseadoL(:,j) = pose_quat2rpy(actualL)+trajectory.CoM(:,j);
%     end
% end
% 
% com_realL = pose_quat2rpy(h.LF_T_CoM(q));
% 
% errorL=com_deseadoL-com_realL;
% 
% errorL_prom=mean(errorL,2);