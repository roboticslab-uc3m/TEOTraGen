clear q_text % Remove this variable if it exists
clc

disp('TEO Humanoid Robot - Plot TEO')
disp('Robotics Lab, Universidad Carlos III de Madrid.')
disp('<a href = "http://roboticslab.uc3m.es">http://roboticslab.uc3m.es</a>')
disp(' ')


if ~exist('q')
    disp('There is not joint values to plot (No "q": variable). TEO will be plot only in default position')
    selection=input('Do you want to continue? y/n [y]: ','s');
    if (selection=='n' | selection=='N')
        return
    elseif  (selection=='y' | selection=='Y')
        disp('Please wait some seconds...')
        disp(' ')
        break
    else
        disp(['Error: ', selection, ' is not an option. TEO will be plot anyway']);
        disp(' ')
        disp('Please wait some seconds...')
        disp(' ')
    end
else
    disp(' ')
    disp('Please wait some seconds...')
    disp(' ')
end;





% LIBRERIA

TEO = TEO_structure('numeric', 'rad', 'm');

% MATRIZ DE CONVERSION PARA PLOT
Rotation_Matrix_Plot_Legs =  r2t([0 -1 0;...
                        0 0 1;...
                        -1 0 0]);
                    
Rotation_Matrix_Plot_Arms =  r2t([1 0 0;...
                        0 -1 0;...
                        0 0 -1]);                  

%LEGS
%link_lengths=TEO.legs.link_lengths;
qplot = generate_symbolic_vector('theta', 6);
[right_leg_floating.joint, right_leg_floating.n_joints] = humanoid_leg_floating_DH_parameters(TEO.legs.link_lengths(2:5), qplot);
[left_leg_floating.joint, left_leg_floating.n_joints] = humanoid_leg_floating_DH_parameters(TEO.legs.link_lengths(2:5), qplot);

for i=1:size(qplot,1)
TEO_right_leg(i) = Link(double([right_leg_floating.joint(i).theta-qplot(i), right_leg_floating.joint(i).d, right_leg_floating.joint(i).a, right_leg_floating.joint(i).alpha, 0]));
TEO_left_leg(i) = Link(double([left_leg_floating.joint(i).theta-qplot(i), -left_leg_floating.joint(i).d, left_leg_floating.joint(i).a, left_leg_floating.joint(i).alpha, 0]));
end;

%ARMS
% generate_humanoid_arms_kinematics  ([TEO.chest.link_lengths], [TEO.arms.link_lengths]);
[humanoid_arm.joint, humanoid_arm.n_joints] = humanoid_arm_DH_parameters(TEO.arms.link_lengths(2:3), qplot);

for i=1:size(qplot,1)
TEO_right_arm(i) = Link(double([humanoid_arm.joint(i).theta-qplot(i), humanoid_arm.joint(i).d, humanoid_arm.joint(i).a, humanoid_arm.joint(i).alpha, 0]));
TEO_left_arm(i) = Link(double([humanoid_arm.joint(i).theta-qplot(i), humanoid_arm.joint(i).d, humanoid_arm.joint(i).a, humanoid_arm.joint(i).alpha, 0]));
end;

%TORSO
%generate_humanoid_torso_kinematics ([TEO.waist.link_lengths], [TEO.torso.link_lengths]);
qplot = generate_symbolic_vector('theta', 2);
[humanoid_torso.joint, humanoid_torso.n_joints] = humanoid_torso_DH_parameters(TEO.torso.link_lengths(1), qplot);

for i=1:size(qplot,1)
TEO_torso(i) = Link(double([humanoid_torso.joint(i).theta-qplot(i), humanoid_torso.joint(i).d, humanoid_torso.joint(i).a, humanoid_torso.joint(i).alpha, 0]));
end;

% TEO_right_leg(1) = Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
%     'I', [0, 0.35, 0, 0, 0, 0], ...
%     'r', [0, 0, 0], ...
%     'm', 0, ...
%     'Jm', 200e-6, ...
%     'G', -62.6111, ...
%     'B', 1.48e-3, ...
%     'Tc', [0.395 -0.435], ...
%     'qplotlim', [-160 160]*deg );

TEO_right_leg = SerialLink(TEO_right_leg, 'name', 'TEO_LR', 'base', transl(TEO.legs.right.joint(1).origin)*Rotation_Matrix_Plot_Legs, 'humanoid_structure', TEO);
TEO_left_leg = SerialLink(TEO_left_leg, 'name', 'TEO_LL', 'base', transl(TEO.legs.left.joint(1).origin)*Rotation_Matrix_Plot_Legs, 'humanoid_structure', TEO);

TEO_right_arm = SerialLink(TEO_right_arm, 'name', 'TEO_AR', 'base', transl(TEO.arms.right.joint(1).origin)*Rotation_Matrix_Plot_Arms, 'humanoid_structure', TEO);
TEO_left_arm = SerialLink(TEO_left_arm, 'name', 'TEO_AL', 'base', transl(TEO.arms.left.joint(1).origin)*Rotation_Matrix_Plot_Arms, 'humanoid_structure', TEO);

TEO_torso = SerialLink(TEO_torso, 'name', 'TEO_T', 'base', transl(TEO.origin), 'humanoid_structure', TEO); % NO ES EL ORIGEN, O SI?


TEO_right_leg.plot(zeros(1,6),'nobase','noshadow','nojaxes','noname','nowrist')

set(gca,'XDir','reverse');
set(gca,'YDir','reverse');
axis([-1.25 1.25 -1.25 1.25 -0.5 2])


hold on;
TEO_left_leg.plot(zeros(1,6),'nobase','noshadow','nojaxes','noname','nowrist')%,'nowrist')

TEO_right_arm.plot(zeros(1,6),'nobase','noshadow','nojaxes','noname','nowrist')
TEO_left_arm.plot(zeros(1,6),'nobase','noshadow','nojaxes','noname','nowrist')

TEO_torso.plot(zeros(1,2),'nobase','noshadow','nojaxes','noname','nowrist')
  
%WORLD COORDINATES
    world_coord_length=0.4;
    worldX=line('color','red', 'LineWidth', 2);
    set(worldX,'xdata', [0 world_coord_length], 'ydata', [0 0], 'zdata', [0 0]);
    worldY=line('color','green', 'LineWidth', 2);
    set(worldY,'xdata', [0 0], 'ydata', [0 world_coord_length], 'zdata', [0 0]);
    worldZ=line('color','blue', 'LineWidth', 2);
    set(worldZ,'xdata', [0 0], 'ydata', [0 0], 'zdata', [0 world_coord_length]);
    
    % cones of the axes
    [xc,yc,zc] = cylinder([0 world_coord_length/15]);
    zc(zc==0) = world_coord_length+world_coord_length/15;
    zc(zc==1) = world_coord_length-world_coord_length/15;
    worldX_cone=surface(zc,yc,xc,'FaceColor', [1 0 0],'FaceAlpha', 1,'EdgeColor', 'none');
    worldY_cone=surface(xc,zc,yc,'FaceColor', [0 1 0],'FaceAlpha', 1,'EdgeColor', 'none');
    worldZ_cone=surface(xc,yc,zc,'FaceColor', [0 0 1],'FaceAlpha', 1,'EdgeColor', 'none');
    
hold off;

% RIGHT FOOT SUPPORT MATRIXES
HT_Matrix_Plot_Right_Leg(:,:,1) =  [0 -1 0 0;...
                        0 0 1 -2*TEO.legs.link_lengths(1);...
                        -1 0 0 0;...
                        0 0 0 1];

HT_Matrix_Plot_Right_Leg(:,:,2) =  [0 -1 0 0;...
                        0 0 1 0;...
                        -1 0 0 0;...
                        0 0 0 1];

                    
HT_Matrix_Plot_Torso(:,:,1) =  [1 0 0 0;...
                        0 1 0 -TEO.legs.link_lengths(1);...
                        0 0 1 0;...
                        0 0 0 1];

HT_Matrix_Plot_Torso(:,:,2) =  [1 0 0 0;...
                        0 1 0 TEO.legs.link_lengths(1);...
                        0 0 1 0;...
                        0 0 0 1];
                    
HT_Matrix_Plot_Left_Leg(:,:,1) =  [0 -1 0 0;...
                        0 0 1 0;...
                        -1 0 0 0;...
                        0 0 0 1];
                    
HT_Matrix_Plot_Left_Leg(:,:,2) =  [0 -1 0 0;...
                        0 0 1 2*TEO.legs.link_lengths(1);...
                        -1 0 0 0;...
                        0 0 0 1];

% HT_Matrix_Plot_Right_Arm =  [1 0 0 0;...
%                         0 -1 0 -TEO.legs.link_lengths(1)-TEO.arms.link_lengths(1);...
%                         0 0 -1 TEO.chest.link_lengths(2);...
%                         0 0 0 1];
                    
HT_Matrix_Plot_Right_Arm(:,:,1) =  [0 1 0 0;...
                        1 0 0 -TEO.legs.link_lengths(1)-TEO.arms.link_lengths(1);...
                        0 0 -1 TEO.chest.link_lengths(2);...
                        0 0 0 1];

HT_Matrix_Plot_Right_Arm(:,:,2) =  [0 1 0 0;...
                        1 0 0 TEO.legs.link_lengths(1)-TEO.arms.link_lengths(1);...
                        0 0 -1 TEO.chest.link_lengths(2);...
                        0 0 0 1];
                    
HT_Matrix_Plot_Left_Arm(:,:,1) =  [0 1 0 0;...
                        1 0 0 -TEO.legs.link_lengths(1)+TEO.arms.link_lengths(1);...
                        0 0 -1 TEO.chest.link_lengths(2);...
                        0 0 0 1];
                    
HT_Matrix_Plot_Left_Arm(:,:,2) =  [0 1 0 0;...
                        1 0 0 TEO.legs.link_lengths(1)+TEO.arms.link_lengths(1);...
                        0 0 -1 TEO.chest.link_lengths(2);...
                        0 0 0 1];
                    
                    
% Stop if there is not the 'q' variable
if ~exist('q')
    disp('ERROR!: There is not a "q" variable') 
    return
end;


waist_homogeneous_transform = zeros(4,4,size(q,2));
torso_homogeneous_transform = zeros(4,4,size(q,2));
CoM_homogeneous_transform = zeros(4,4,size(q,2));
TEO_right_leg_base = zeros(4,4,size(q,2));
TEO_left_leg_base = zeros(4,4,size(q,2));
TEO_right_arm_base = zeros(4,4,size(q,2));
TEO_left_arm_base = zeros(4,4,size(q,2));
TEO_torso_base = zeros(4,4,size(q,2));

%Calcular transformaciones antes de mostrar el movimiento
for k=1:size(q,2)
    if k==1
        previous_support=0;
    else
    	previous_support=trajectory.SF(k-1);
    end
    
    if strcmp(data.Leg,'Right Leg')
        %Waist pose
        if ((previous_support<=0)&&(trajectory.SF(k)<=0))
            waist_pose_RPY= pose_quat2rpy(h.RF_T_w(q(:,k)));
            waist_rotation = RPY2Rot_Mat(waist_pose_RPY(4:6));
            waist_homogeneous_transform(:,:,k) = rt2tr(waist_rotation, waist_pose_RPY(1:3));

            %Torso pose
            torso_homogeneous_transform(:,:,k)=transl(0, 0, TEO.waist.link_lengths(2))*waist_homogeneous_transform(:,:,k);

            %Arms pose
            CoM_pose_RPY= pose_quat2rpy(h.RF_T_CoM(q(:,k)));
            CoM_rotation = RPY2Rot_Mat(CoM_pose_RPY(4:6));
            CoM_homogeneous_transform(:,:,k) = rt2tr(CoM_rotation, CoM_pose_RPY(1:3)); 

            %Select legs,arms and torso bases
            TEO_right_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Leg(:,:,1);
            TEO_left_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Leg(:,:,1);

            TEO_right_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Arm(:,:,1);
            TEO_left_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Arm(:,:,1);

            TEO_torso_base(:,:,k) = torso_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Torso(:,:,1);
            
        elseif (trajectory.SF(k)>0)
            %Waist change
            if (previous_support<=0)
                %Waist Difference
                previous_RF_waist_pose_RPY= pose_quat2rpy(h.RF_T_w(q(:,k-1)));
                previous_RF_waist_rotation = RPY2Rot_Mat(previous_RF_waist_pose_RPY(4:6));
                previous_RF_waist_homogeneous_transform = rt2tr(previous_RF_waist_rotation, previous_RF_waist_pose_RPY(1:3));
                                
                previous_LF_waist_pose_RPY= pose_quat2rpy(h.LF_T_w(q(:,k-1)));
                previous_LF_waist_rotation = RPY2Rot_Mat(previous_LF_waist_pose_RPY(4:6));
                previous_LF_waist_homogeneous_transform = rt2tr(previous_LF_waist_rotation, previous_LF_waist_pose_RPY(1:3));
                
                waist_difference_homogeneous_transform =  previous_RF_waist_homogeneous_transform/(previous_LF_waist_homogeneous_transform);
                
                %CoM Difference                              
                previous_RF_CoM_pose_RPY= pose_quat2rpy(h.RF_T_CoM(q(:,k)));
                previous_RF_CoM_rotation = RPY2Rot_Mat(previous_RF_CoM_pose_RPY(4:6));
                previous_RF_CoM_homogeneous_transform = rt2tr(previous_RF_CoM_rotation, previous_RF_CoM_pose_RPY(1:3)); 
                
                previous_LF_CoM_pose_RPY= pose_quat2rpy(h.LF_T_CoM(q(:,k)));
                previous_LF_CoM_rotation = RPY2Rot_Mat(previous_LF_CoM_pose_RPY(4:6));
                previous_LF_CoM_homogeneous_transform = rt2tr(previous_LF_CoM_rotation, previous_LF_CoM_pose_RPY(1:3)); 
                
                CoM_difference_homogeneous_transform =  previous_RF_CoM_homogeneous_transform/(previous_LF_CoM_homogeneous_transform);
            end

            %Waist pose
            waist_pose_RPY= pose_quat2rpy(h.LF_T_w(q(:,k)));
            waist_rotation = RPY2Rot_Mat(waist_pose_RPY(4:6));
            waist_homogeneous_transform(:,:,k) = waist_difference_homogeneous_transform*rt2tr(waist_rotation, waist_pose_RPY(1:3));

            %Torso pose
            torso_homogeneous_transform(:,:,k)=transl(0, 0, TEO.waist.link_lengths(2))*waist_homogeneous_transform(:,:,k);

            %Arms pose
            CoM_pose_RPY= pose_quat2rpy(h.LF_T_CoM(q(:,k)));
            CoM_rotation = RPY2Rot_Mat(CoM_pose_RPY(4:6));
            CoM_homogeneous_transform(:,:,k) = rt2tr(CoM_rotation, CoM_pose_RPY(1:3));    

            TEO_right_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Leg(:,:,1);
            TEO_left_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Leg(:,:,1);

            TEO_right_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Arm(:,:,1);
            TEO_left_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Arm(:,:,1);

            TEO_torso_base(:,:,k) = torso_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Torso(:,:,1);            
        end
    elseif strcmp(data.Leg,'Left Leg')
        %Waist pose
        waist_pose_RPY= pose_quat2rpy(h.LF_T_w(q(:,k)));
        waist_rotation = RPY2Rot_Mat(waist_pose_RPY(4:6));
        waist_homogeneous_transform(:,:,k) = rt2tr(waist_rotation, waist_pose_RPY(1:3));

        %Torso pose
        torso_homogeneous_transform(:,:,k)=transl(0, 0, TEO.waist.link_lengths(2))*waist_homogeneous_transform(:,:,k);

        %Arms pose
        CoM_pose_RPY= pose_quat2rpy(h.LF_T_CoM(q(:,k)));
        CoM_rotation = RPY2Rot_Mat(CoM_pose_RPY(4:6));
        CoM_homogeneous_transform(:,:,k) = rt2tr(CoM_rotation, CoM_pose_RPY(1:3));    

        TEO_right_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Leg(:,:,2);
        TEO_left_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Leg(:,:,2);

        TEO_right_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Arm(:,:,2);
        TEO_left_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Arm(:,:,2);

        TEO_torso_base(:,:,k) = torso_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Torso(:,:,2);        
    else
        disp('ERROR!: Wrong trajectory Foot option') 
    end
        
        
end
    

for k=1:size(q,2)
    %Plot the number of q
    if exist('q_text')
        delete(q_text);
    end;
    
    q_text = text(0,0.5,1.5,strcat('qt=',num2str(k)),'Color','g');
    
    TEO_right_leg.base = TEO_right_leg_base(:,:,k);
    TEO_left_leg.base = TEO_left_leg_base(:,:,k);

    TEO_right_arm.base = TEO_right_arm_base(:,:,k);
    TEO_left_arm.base = TEO_left_arm_base(:,:,k);

    TEO_torso.base = TEO_torso_base(:,:,k);
    
    %%%Plot
    %Legs
    TEO_right_leg.plot(q(1:6,k)');
    TEO_left_leg.plot(q(7:12,k)'); 
    
    %Torso
    TEO_torso.plot(q(13:14,k)');
    
    %Arms
    TEO_right_arm.plot(q(15:20,k)');
    TEO_left_arm.plot(q(21:26,k)');
    
    
%     %Hip
%     if exist('hiplink')
%         delete(hiplink);
%     end;
%     
%     waist_pose_plot = waist_homogeneous_transform(:,:,k)*Rotation_Matrix_Plot_Legs;
%     
%     hiplink = line(TEO_right_leg.lineopt{:});
%     xlip(1)=TEO_right_leg.base(1,4);
%     ylip(1)=TEO_right_leg.base(2,4);
%     zlip(1)=TEO_right_leg.base(3,4);
%     xlip(2)=waist_pose_plot(1,4);
%     ylip(2)=waist_pose_plot(2,4);
%     zlip(2)=waist_pose_plot(3,4);
%     xlip(3)=TEO_left_leg.base(1,4);
%     ylip(3)=TEO_left_leg.base(2,4);
%     zlip(3)=TEO_left_leg.base(3,4);  
%     set(hiplink,'xdata', xlip, 'ydata', ylip, 'zdata', zlip);
      
    
%     drawnow
    
end