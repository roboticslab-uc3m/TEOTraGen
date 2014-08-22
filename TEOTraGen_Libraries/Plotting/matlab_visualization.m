%% MATLAB visualization
% This function plots a humanoid movement using Robotics Toolbox of Peter
% Corke.

function matlab_visualization (trajectory, q, humanoid_structure, h)
% Author: Domingo Esteban

q0 = q(:,1);
ndata = size(q,2);

% MATRIZ DE CONVERSION PARA PLOT
Rotation_Matrix_Plot_Legs =  r2t([0 -1 0;...
                        0 0 1;...
                        -1 0 0]);
%                     
Rotation_Matrix_Plot_Arms =  r2t([1 0 0;...
                        0 -1 0;...
                        0 0 -1]);          
                      
                      
% Rotation_Matrix_Plot_Legs = eye(4);
Rotation_Matrrix_Plot_Arms = eye(4);

% LEGS
qplot = generate_symbolic_vector('theta', 6);
[right_leg_floating.joint, right_leg_floating.n_joints] = humanoid_leg_floating_DH_parameters(humanoid_structure.legs.link_lengths(2:5), qplot);
[left_leg_floating.joint, left_leg_floating.n_joints] = humanoid_leg_floating_DH_parameters(humanoid_structure.legs.link_lengths(2:5), qplot);

for i = 1:size(qplot,1)
  humanoid_right_leg(i) = Link(double([right_leg_floating.joint(i).theta-qplot(i), right_leg_floating.joint(i).d, right_leg_floating.joint(i).a, right_leg_floating.joint(i).alpha, 0]));
  humanoid_left_leg(i) = Link(double([left_leg_floating.joint(i).theta-qplot(i), -left_leg_floating.joint(i).d, left_leg_floating.joint(i).a, left_leg_floating.joint(i).alpha, 0]));
end;

% ARMS
[humanoid_arm.joint, humanoid_arm.n_joints] = humanoid_arm_DH_parameters(humanoid_structure.arms.link_lengths(2:3), qplot);

for i = 1:size(qplot,1)
 humanoid_right_arm(i) = Link(double([humanoid_arm.joint(i).theta-qplot(i), humanoid_arm.joint(i).d, humanoid_arm.joint(i).a, humanoid_arm.joint(i).alpha, 0]));
 humanoid_left_arm(i) = Link(double([humanoid_arm.joint(i).theta-qplot(i), humanoid_arm.joint(i).d, humanoid_arm.joint(i).a, humanoid_arm.joint(i).alpha, 0]));
end;

%TORSO
qplot = generate_symbolic_vector('theta', 2);
[humanoid_waistjoint, humanoid_waistn_joints] = humanoid_torso_DH_parameters(humanoid_structure.torso.link_lengths(1), qplot);

for i = 1:size(qplot,1)
  humanoid_torso(i) = Link(double([humanoid_waistjoint(i).theta-qplot(i), humanoid_waistjoint(i).d, humanoid_waistjoint(i).a, humanoid_waistjoint(i).alpha, 0]));
end;

% Humanoid parts classes
humanoid_right_leg = SerialLink(humanoid_right_leg, 'name', 'TEO_RL');
humanoid_left_leg = SerialLink(humanoid_left_leg, 'name', 'TEO_LL');
humanoid_right_arm = SerialLink(humanoid_right_arm, 'name', 'TEO_RA');
humanoid_left_arm = SerialLink(humanoid_left_arm, 'name', 'TEO_LA');
humanoid_torso = SerialLink(humanoid_torso, 'name', 'TEO_T');

% Prealllocate some variables
inertial_right_leg_base = zeros(4,4,ndata);
inertial_left_leg_base = zeros(4,4,ndata);
inertial_com_base = zeros(4,4,ndata);
inertial_waist_base = zeros(4,4,ndata);
inertial_right_arm_base = zeros(4,4,ndata);
inertial_left_arm_base = zeros(4,4,ndata);
inertial_rf = zeros(4,4,ndata);
inertial_lf = zeros(4,4,ndata);

% Constant transformations
w_T_rl_base = transl([0; humanoid_structure.legs.right.joint(1).origin(2); 0]);
w_T_ll_base = transl([0; humanoid_structure.legs.left.joint(1).origin(2); 0]);
com_T_ra_base = transl([humanoid_structure.torso.link_lengths(1); -humanoid_structure.arms.link_lengths(1);humanoid_structure.torso.link_lengths(2)]);
com_T_la_base = transl([humanoid_structure.torso.link_lengths(1); humanoid_structure.arms.link_lengths(1);humanoid_structure.torso.link_lengths(2)]);

% For plotting feet
rf_xmin = humanoid_structure.legs.right.foot.limits.x(1);
rf_xmax = humanoid_structure.legs.right.foot.limits.x(2);
rf_ymin = humanoid_structure.legs.right.foot.limits.y(1);
rf_ymax = humanoid_structure.legs.right.foot.limits.y(2);
lf_xmin = humanoid_structure.legs.right.foot.limits.x(1);
lf_xmax = humanoid_structure.legs.left.foot.limits.x(2);
lf_ymin = humanoid_structure.legs.left.foot.limits.y(1);
lf_ymax = humanoid_structure.legs.left.foot.limits.y(2);
feet_height = 0.01;

for ii = 1:ndata
  if ii == 1
    w_T_rf = pose_quat2tr(real(h.w_T_RF(q(:,ii))));
    w_T_lf = pose_quat2tr(real(h.w_T_LF(q(:,ii))));
    w_T_com = pose_quat2tr(real(h.w_T_CoM(q(:,ii))));
    inertial_waist_base(:,:,ii) = transl([(w_T_rf(1:2,4) + w_T_lf(1:2,4))/2; -min(w_T_rf(3,4), w_T_lf(3,4))]);
    inertial_rf(:,:,ii) = multiply_homogeneous_matrix({inertial_waist_base(:,:,ii) w_T_rf});
    inertial_lf(:,:,ii) = multiply_homogeneous_matrix({inertial_waist_base(:,:,ii) w_T_lf});
    
    inertial_right_leg_base(:,:,ii) = multiply_homogeneous_matrix({inertial_waist_base(:,:,ii) w_T_rl_base Rotation_Matrix_Plot_Legs});
    inertial_left_leg_base(:,:,ii) = multiply_homogeneous_matrix({inertial_waist_base(:,:,ii) w_T_ll_base Rotation_Matrix_Plot_Legs});
    inertial_com_base(:,:,ii) = multiply_homogeneous_matrix({inertial_waist_base(:,:,ii) w_T_com});
  else
  
    if (trajectory.SF(ii) == 0) || (trajectory.SF(ii) == -1)
      inertial_rf(:,:,ii) = inertial_rf(:,:,ii-1);
      w_T_rf = pose_quat2tr(real(h.w_T_RF(q(:,ii))));
      w_T_lf = pose_quat2tr(real(h.w_T_LF(q(:,ii))));
      w_T_com = pose_quat2tr(real(h.w_T_CoM(q(:,ii))));
      inertial_waist_base(:,:,ii) = multiply_homogeneous_matrix({inertial_rf(:,:,ii) invert_homogeneous_matrix(w_T_rf)});
      
      inertial_lf(:,:,ii) = multiply_homogeneous_matrix({inertial_waist_base(:,:,ii) w_T_lf});  
      inertial_right_leg_base(:,:,ii) = multiply_homogeneous_matrix({inertial_waist_base(:,:,ii) w_T_rl_base Rotation_Matrix_Plot_Legs});
      inertial_left_leg_base(:,:,ii) = multiply_homogeneous_matrix({inertial_waist_base(:,:,ii) w_T_ll_base Rotation_Matrix_Plot_Legs});
      inertial_com_base(:,:,ii) = multiply_homogeneous_matrix({inertial_waist_base(:,:,ii) w_T_com});
    elseif trajectory.SF(ii) == 1
      inertial_lf(:,:,ii) = inertial_lf(:,:,ii-1);
      w_T_rf = pose_quat2tr(real(h.w_T_RF(q(:,ii))));
      w_T_lf = pose_quat2tr(real(h.w_T_LF(q(:,ii))));
      w_T_com = pose_quat2tr(real(h.w_T_CoM(q(:,ii))));
      inertial_waist_base(:,:,ii) = multiply_homogeneous_matrix({inertial_lf(:,:,ii) invert_homogeneous_matrix(w_T_lf)});
      
      inertial_rf(:,:,ii) = multiply_homogeneous_matrix({inertial_waist_base(:,:,ii) w_T_rf});  
      inertial_right_leg_base(:,:,ii) = multiply_homogeneous_matrix({inertial_waist_base(:,:,ii) w_T_rl_base Rotation_Matrix_Plot_Legs});
      inertial_left_leg_base(:,:,ii) = multiply_homogeneous_matrix({inertial_waist_base(:,:,ii) w_T_ll_base Rotation_Matrix_Plot_Legs});
      inertial_com_base(:,:,ii) = multiply_homogeneous_matrix({inertial_waist_base(:,:,ii) w_T_com});
    end
  
  end
  inertial_right_arm_base(:,:,ii) = multiply_homogeneous_matrix({inertial_com_base(:,:,ii) com_T_ra_base Rotation_Matrix_Plot_Arms});
  inertial_left_arm_base(:,:,ii) = multiply_homogeneous_matrix({inertial_com_base(:,:,ii) com_T_la_base Rotation_Matrix_Plot_Arms});  
  
end

figure(50), title('TEO Humanoid Robot - Universidad Carlos III de Madrid','color','blue','FontSize',16)

humanoid_right_leg.base = inertial_right_leg_base(:,:,1); 
humanoid_left_leg.base = inertial_left_leg_base(:,:,1);
humanoid_torso.base = inertial_com_base(:,:,1);
humanoid_right_arm.base = inertial_right_arm_base(:,:,1);
humanoid_left_arm.base = inertial_left_arm_base(:,:,1);

plot(humanoid_right_leg, q(1:6,1)','nobase','noshadow','nojaxes','noname','nowrist')

set(gca,'XDir','reverse');
set(gca,'YDir','reverse');
axis([-1.25 1.25 -1.25 1.25 -0.5 2])

hold on
  plot(humanoid_left_leg, q(7:12,1)','nobase','noshadow','nojaxes','noname','nowrist')
  plot(humanoid_torso, q(13:14,1)','nobase','noshadow','nojaxes','noname','nowrist')
  plot(humanoid_right_arm, q(15:20,1)','nobase','noshadow','nojaxes','noname','nowrist')
  plot(humanoid_left_arm, q(21:26,1)','nobase','noshadow','nojaxes','noname','nowrist')

  % WORLD COORDINATES
    world_coord_length = 0.4;
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

for ii = 2:ndata
  % Plot the number of q
  if exist('q_text')
    delete(q_text);
  end;

  q_text = text(0,0.5,1.5,strcat('qt=',num2str(ii)),'Color','g');

  % Change humanoid_parts bases w.r.t. to inertial frame
  humanoid_right_leg.base = inertial_right_leg_base(:,:,ii);
  humanoid_left_leg.base = inertial_left_leg_base(:,:,ii);
  humanoid_torso.base = inertial_com_base(:,:,ii);
  humanoid_right_arm.base = inertial_right_arm_base(:,:,ii);
  humanoid_left_arm.base = inertial_left_arm_base(:,:,ii);

  % Plot the humanoid parts
  humanoid_right_leg.plot( q(1:6,ii)')
  humanoid_left_leg.plot( q(7:12,ii)')
  plot(humanoid_torso, q(13:14,ii)')
  plot(humanoid_right_arm, q(15:20,ii)')
  plot(humanoid_left_arm, q(21:26,ii)')
  
  
% HIP
    if exist('hiplink','var')
      delete(hiplink);  
    end
    hiplink = line(humanoid_right_leg.lineopt{:});
    xlip(1)=inertial_right_leg_base(1,4,ii);
    ylip(1)=inertial_right_leg_base(2,4,ii);
    zlip(1)=inertial_right_leg_base(3,4,ii); 
    xlip(2)=inertial_left_leg_base(1,4,ii);
    ylip(2)=inertial_left_leg_base(2,4,ii);
    zlip(2)=inertial_left_leg_base(3,4,ii);
    set(hiplink,'xdata', xlip, 'ydata', ylip, 'zdata', zlip);

% WAIST
    if exist('waistlink','var')
      delete(waistlink);  
    end
    waistlink=line(humanoid_torso.lineopt{:});
    xwaist(1)=inertial_waist_base(1,4,ii);
    ywaist(1)=inertial_waist_base(2,4,ii);
    zwaist(1)=inertial_waist_base(3,4,ii);
    xwaist(2)=inertial_com_base(1,4,ii);
    ywaist(2)=inertial_com_base(2,4,ii);
    zwaist(2)=inertial_com_base(3,4,ii);
    set(waistlink,'xdata', xwaist, 'ydata', ywaist, 'zdata', zwaist);
    
% CLAVICLE
    if exist('claviclelink','var')
      delete(claviclelink);  
    end
    claviclelink=line(humanoid_right_arm.lineopt{:});
    xclavicle(1)=inertial_right_arm_base(1,4,ii);
    yclavicle(1)=inertial_right_arm_base(2,4,ii);
    zclavicle(1)=inertial_right_arm_base(3,4,ii); 
    xclavicle(2)=inertial_left_arm_base(1,4,ii);
    yclavicle(2)=inertial_left_arm_base(2,4,ii);
    zclavicle(2)=inertial_left_arm_base(3,4,ii);   
    set(claviclelink,'xdata', xclavicle, 'ydata', yclavicle, 'zdata', zclavicle);
  
% FEET
    if exist('Right_Foot_plotk','var')
      delete(Right_Foot_plot);  
    end
    if exist('Left_Foot_plot','var')
      delete(Left_Foot_plot);  
    end

%     Foot_length = 0.2;
%     Foot_width = 0.1;
%     SEPARACION_R=-0.113;
%     SEPARACION_L=0.113;
    
   
    Right_Foot_plot = patch([rf_xmax+inertial_lf(1,4,ii) rf_xmin+inertial_lf(1,4,ii) rf_xmin+inertial_lf(1,4,ii) rf_xmax+inertial_lf(1,4,ii)], [rf_ymax+inertial_rf(2,4,ii) rf_ymax+inertial_rf(2,4,ii) rf_ymin+inertial_rf(2,4,ii) rf_ymin+inertial_rf(2,4,ii)], [0 0 0 0], ...
    'FaceColor', 'k', 'FaceAlpha', 0.5);
    Left_Foot_plot =  patch([lf_xmax+inertial_lf(1,4,ii) lf_xmin+inertial_lf(1,4,ii) lf_xmin+inertial_lf(1,4,ii) lf_xmax+inertial_lf(1,4,ii)], [lf_ymax+inertial_lf(2,4,ii) lf_ymax+inertial_lf(2,4,ii) lf_ymin+inertial_lf(2,4,ii) lf_ymin+inertial_lf(2,4,ii)], [0 0 0 0], ...
    'FaceColor', 'k', 'FaceAlpha', 0.5);
    
  drawnow
end



% 
% TEO_right_arm.plot(zeros(1,6),'nobase','noshadow','nojaxes','noname','nowrist')
% TEO_left_arm.plot(zeros(1,6),'nobase','noshadow','nojaxes','noname','nowrist')
% 
% TEO_torso.plot(zeros(1,2),'nobase','noshadow','nojaxes','noname','nowrist')

% 
% % 
% % %%%DOMINGOOOOOOOOOOOOOOOOOOOOOO
% % %HIP
% %     humanoid_structure.hip.origin(1)=humanoid_structure.legs.right.joint(1).origin(1);
% %     humanoid_structure.hip.origin(2)=humanoid_structure.legs.right.joint(1).origin(2)+humanoid_structure.legs.link_lengths(1);
% %     humanoid_structure.hip.origin(3)=humanoid_structure.legs.right.joint(1).origin(3);
% %     
% %     hiplink = line(humanoid_right_leg.lineopt{:});
% %     xlip(1)=humanoid_structure.legs.right.joint(1).origin(1);
% %     ylip(1)=humanoid_structure.legs.right.joint(1).origin(2);
% %     zlip(1)=humanoid_structure.legs.right.joint(1).origin(3);
% %     xlip(2)=humanoid_structure.hip.origin(1);
% %     ylip(2)=humanoid_structure.hip.origin(2);
% %     zlip(2)=humanoid_structure.hip.origin(3);    
% %     xlip(3)=humanoid_structure.legs.left.joint(1).origin(1);
% %     ylip(3)=humanoid_structure.legs.left.joint(1).origin(2);
% %     zlip(3)=humanoid_structure.legs.left.joint(1).origin(3);    
% %     set(hiplink,'xdata', xlip, 'ydata', ylip, 'zdata', zlip);
% %     
% % %WAIST
% %     waistlink=line(TEO_torso.lineopt{:});
% %     xwaist(1)=humanoid_structure.hip.origin(1);
% %     ywaist(1)=humanoid_structure.hip.origin(2);
% %     zwaist(1)=humanoid_structure.hip.origin(3);
% %     xwaist(2)=humanoid_structure.waist.joint(1).origin(1);
% %     ywaist(2)=humanoid_structure.waist.joint(1).origin(2);
% %     zwaist(2)=humanoid_structure.waist.joint(1).origin(3);
% %     set(waistlink,'xdata', xwaist, 'ydata', ywaist, 'zdata', zwaist);
% % 
% % %CLAVICLE
% %     humanoid_structure.clavicle.origin(1)=humanoid_structure.arms.right.joint(1).origin(1);
% %     humanoid_structure.clavicle.origin(2)=humanoid_structure.arms.right.joint(1).origin(2)+humanoid_structure.arms.link_lengths(1);
% %     humanoid_structure.clavicle.origin(3)=humanoid_structure.arms.right.joint(1).origin(3);
% %     
% %     claviclelink=line(TEO_right_arm.lineopt{:});
% %     xclavicle(1)=humanoid_structure.arms.right.joint(1).origin(1);
% %     yclavicle(1)=humanoid_structure.arms.right.joint(1).origin(2);
% %     zclavicle(1)=humanoid_structure.arms.right.joint(1).origin(3);
% %     xclavicle(2)=humanoid_structure.clavicle.origin(1);
% %     yclavicle(2)=humanoid_structure.clavicle.origin(2);
% %     zclavicle(2)=humanoid_structure.clavicle.origin(3);    
% %     xclavicle(3)=humanoid_structure.arms.left.joint(1).origin(1);
% %     yclavicle(3)=humanoid_structure.arms.left.joint(1).origin(2);
% %     zclavicle(3)=humanoid_structure.arms.left.joint(1).origin(3);    
% %     set(claviclelink,'xdata', xclavicle, 'ydata', yclavicle, 'zdata', zclavicle);
% %     
% % %TORSO
% %     torsolink=line(TEO_torso.lineopt{:});
% %     xtorso(1)=humanoid_structure.waist.joint(1).origin(1);
% %     ytorso(1)=humanoid_structure.waist.joint(1).origin(2);
% %     ztorso(1)=humanoid_structure.waist.joint(1).origin(3);
% %     xtorso(2)=humanoid_structure.clavicle.origin(1);
% %     ytorso(2)=humanoid_structure.clavicle.origin(2);
% %     ztorso(2)=humanoid_structure.clavicle.origin(3);
% %     set(torsolink,'xdata', xtorso, 'ydata', ytorso, 'zdata', ztorso);
% % 
% % %FEET
% %     % draw the robot's body
% %     Foot_length=0.2;
% %     Foot_width=0.1;
% %     SEPARACION_R=-0.113;
% %     SEPARACION_L=0.113;
% %     Right_Foot_plot = patch([Foot_length/2 -Foot_length/2 -Foot_length/2 Foot_length/2], [Foot_width/2+SEPARACION_R Foot_width/2+SEPARACION_R -Foot_width/2+SEPARACION_R -Foot_width/2+SEPARACION_R], [0 0 0 0], ...
% %     'FaceColor', 'k', 'FaceAlpha', 0.5);
% %     Left_Foot_plot = patch([Foot_length/2 -Foot_length/2 -Foot_length/2 Foot_length/2], [Foot_width/2+SEPARACION_L Foot_width/2+SEPARACION_L -Foot_width/2+SEPARACION_L -Foot_width/2+SEPARACION_L], [0 0 0 0], ...
% %     'FaceColor', 'k', 'FaceAlpha', 0.5);

    
%WORLD COORDINATES
%     world_coord_length=0.4;
%     worldX=line('color','red', 'LineWidth', 2);
%     set(worldX,'xdata', [0 world_coord_length], 'ydata', [0 0], 'zdata', [0 0]);
%     worldY=line('color','green', 'LineWidth', 2);
%     set(worldY,'xdata', [0 0], 'ydata', [0 world_coord_length], 'zdata', [0 0]);
%     worldZ=line('color','blue', 'LineWidth', 2);
%     set(worldZ,'xdata', [0 0], 'ydata', [0 0], 'zdata', [0 world_coord_length]);
%     
%     % cones of the axes
%     [xc,yc,zc] = cylinder([0 world_coord_length/15]);
%     zc(zc==0) = world_coord_length+world_coord_length/15;
%     zc(zc==1) = world_coord_length-world_coord_length/15;
%     worldX_cone=surface(zc,yc,xc,'FaceColor', [1 0 0],'FaceAlpha', 1,'EdgeColor', 'none');
%     worldY_cone=surface(xc,zc,yc,'FaceColor', [0 1 0],'FaceAlpha', 1,'EdgeColor', 'none');
%     worldZ_cone=surface(xc,yc,zc,'FaceColor', [0 0 1],'FaceAlpha', 1,'EdgeColor', 'none');
%     
% hold off;

% % walk!
% % k = 1;
% % while 1
% %     legs(1).plot( gait(qcycle, k, 0,   0), plotopt);
% %     legs(2).plot( gait(qcycle, k, 100, 0), plotopt);
% %     legs(3).plot( gait(qcycle, k, 200, 1), plotopt);
% %     legs(4).plot( gait(qcycle, k, 300, 1), plotopt);
% %     drawnow
% %     k = k+1;
% % end
% 
% 
% % RIGHT FOOT SUPPORT MATRIXES
% HT_Matrix_Plot_Right_Leg(:,:,1) =  [0 -1 0 0;...
%                         0 0 1 -2*humanoid_structure.legs.link_lengths(1);...
%                         -1 0 0 0;...
%                         0 0 0 1];
% 
% HT_Matrix_Plot_Right_Leg(:,:,2) =  [0 -1 0 0;...
%                         0 0 1 0;...
%                         -1 0 0 0;...
%                         0 0 0 1];
% 
%                     
% HT_Matrix_Plot_Torso(:,:,1) =  [1 0 0 0;...
%                         0 1 0 -humanoid_structure.legs.link_lengths(1);...
%                         0 0 1 0;...
%                         0 0 0 1];
% 
% HT_Matrix_Plot_Torso(:,:,2) =  [1 0 0 0;...
%                         0 1 0 humanoid_structure.legs.link_lengths(1);...
%                         0 0 1 0;...
%                         0 0 0 1];
%                     
% HT_Matrix_Plot_Left_Leg(:,:,1) =  [0 -1 0 0;...
%                         0 0 1 0;...
%                         -1 0 0 0;...
%                         0 0 0 1];
%                     
% HT_Matrix_Plot_Left_Leg(:,:,2) =  [0 -1 0 0;...
%                         0 0 1 2*humanoid_structure.legs.link_lengths(1);...
%                         -1 0 0 0;...
%                         0 0 0 1];
% 
% % HT_Matrix_Plot_Right_Arm =  [1 0 0 0;...
% %                         0 -1 0 -humanoid_structure.legs.link_lengths(1)-humanoid_structure.arms.link_lengths(1);...
% %                         0 0 -1 humanoid_structure.chest.link_lengths(2);...
% %                         0 0 0 1];
%                     
% HT_Matrix_Plot_Right_Arm(:,:,1) =  [0 1 0 0;...
%                         1 0 0 -humanoid_structure.legs.link_lengths(1)-humanoid_structure.arms.link_lengths(1);...
%                         0 0 -1 humanoid_structure.chest.link_lengths(2);...
%                         0 0 0 1];
% 
% HT_Matrix_Plot_Right_Arm(:,:,2) =  [0 1 0 0;...
%                         1 0 0 humanoid_structure.legs.link_lengths(1)-humanoid_structure.arms.link_lengths(1);...
%                         0 0 -1 humanoid_structure.chest.link_lengths(2);...
%                         0 0 0 1];
%                     
% HT_Matrix_Plot_Left_Arm(:,:,1) =  [0 1 0 0;...
%                         1 0 0 -humanoid_structure.legs.link_lengths(1)+humanoid_structure.arms.link_lengths(1);...
%                         0 0 -1 humanoid_structure.chest.link_lengths(2);...
%                         0 0 0 1];
%                     
% HT_Matrix_Plot_Left_Arm(:,:,2) =  [0 1 0 0;...
%                         1 0 0 humanoid_structure.legs.link_lengths(1)+humanoid_structure.arms.link_lengths(1);...
%                         0 0 -1 humanoid_structure.chest.link_lengths(2);...
%                         0 0 0 1];
%                     
%                     
% % Stop if there is not the 'q' variable
% if ~exist('q')
%     disp('ERROR!: There is not a "q" variable') 
%     return
% end;
% 
% 
% waist_homogeneous_transform=zeros(4,4,size(q,2));
% torso_homogeneous_transform=zeros(4,4,size(q,2));
% CoM_homogeneous_transform=zeros(4,4,size(q,2));
% 
% waist_homogeneous_transform_left=zeros(4,4,size(q,2));
% 
% humanoid_right_leg_base=zeros(4,4,size(q,2));
% humanoid_left_leg_base=zeros(4,4,size(q,2));
% TEO_right_arm_base=zeros(4,4,size(q,2));
% TEO_left_arm_base=zeros(4,4,size(q,2));
% TEO_torso_base=zeros(4,4,size(q,2));
% 
% %Calculate transformations before movement
% for k=1:size(q,2)
%     
%     if trajectory.SF(k)==0
%         %Waist pose
%         waist_pose_RPY= pose_quat2rpy(h.RF_T_w(q(:,k)));
%         waist_rotation = RPY2Rot_Mat(waist_pose_RPY(4:6));
%         waist_homogeneous_transform(:,:,k) = rt2tr(waist_rotation, waist_pose_RPY(1:3));
%         
%         %Waist LEFT
%         waist_pose_RPY_left= pose_quat2rpy(h.LF_T_w(q(:,k)));
%         waist_rotation_left = RPY2Rot_Mat(waist_pose_RPY_left(4:6));
%         waist_homogeneous_transform_left(:,:,k) = rt2tr(waist_rotation_left, waist_pose_RPY_left(1:3));   
%         
%         %Obtain Deduct CoM difference between RF_T_w and RL_T_w (only at
%         %the beginning)
%         if k==1
%             waist_pose_RPY_difference=pose_quat2rpy(h.w_T_LF(q(:,k)))+waist_pose_RPY_left;
%             waist_pose_RPY_difference(4:6)=0;
%         end
%         
%         waist_rotation_difference = RPY2Rot_Mat(waist_pose_RPY_difference(4:6));
%         waist_homogeneous_transform_difference(:,:,k) = rt2tr(waist_rotation_difference, waist_pose_RPY_difference(1:3));       
% 
%         %Torso pose
%         torso_homogeneous_transform(:,:,k)=transl(0, 0, humanoid_structure.waist.link_lengths(2))*waist_homogeneous_transform(:,:,k);
%         
%         %Arms pose
%         CoM_pose_RPY= pose_quat2rpy(h.RF_T_CoM(q(:,k)));
%         CoM_rotation = RPY2Rot_Mat(CoM_pose_RPY(4:6));
%         CoM_homogeneous_transform(:,:,k) = rt2tr(CoM_rotation, CoM_pose_RPY(1:3)); 
%         
%         %Select legs,arms and torso bases
%         humanoid_right_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Leg(:,:,1);
%         %humanoid_left_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Leg(:,:,1);
%         humanoid_left_leg_base(:,:,k) = waist_homogeneous_transform_difference(:,:,k)*waist_homogeneous_transform_left(:,:,k)*HT_Matrix_Plot_Left_Leg(:,:,2);
% 
%         TEO_right_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Arm(:,:,1);
%         TEO_left_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Arm(:,:,1);
% 
%         TEO_torso_base(:,:,k) = torso_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Torso(:,:,1);
%         
%     
%     elseif (trajectory.SF(k)==-1)
%         %Waist pose
%         waist_pose_RPY= pose_quat2rpy(h.RF_T_w(q(:,k)));
%         waist_rotation = RPY2Rot_Mat(waist_pose_RPY(4:6));
%         waist_homogeneous_transform(:,:,k) = rt2tr(waist_rotation, waist_pose_RPY(1:3));
% 
%         %Torso pose
%         torso_homogeneous_transform(:,:,k)=transl(0, 0, humanoid_structure.waist.link_lengths(2))*waist_homogeneous_transform(:,:,k);
% 
%         %Arms pose
%         CoM_pose_RPY= pose_quat2rpy(h.RF_T_CoM(q(:,k)));
%         CoM_rotation = RPY2Rot_Mat(CoM_pose_RPY(4:6));
%         CoM_homogeneous_transform(:,:,k) = rt2tr(CoM_rotation, CoM_pose_RPY(1:3)); 
%         
%         %Select legs,arms and torso bases
%         humanoid_right_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Leg(:,:,1);
%         humanoid_left_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Leg(:,:,1);
% 
%         TEO_right_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Arm(:,:,1);
%         TEO_left_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Arm(:,:,1);
% 
%         TEO_torso_base(:,:,k) = torso_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Torso(:,:,1);
%         
%     elseif trajectory.SF(k)==1
%         %Waist pose
%         waist_pose_RPY= pose_quat2rpy(h.LF_T_w(q(:,k)));
%         waist_rotation = RPY2Rot_Mat(waist_pose_RPY(4:6));
%         waist_homogeneous_transform(:,:,k) = rt2tr(waist_rotation, waist_pose_RPY(1:3));
% 
%         %Torso pose
%         torso_homogeneous_transform(:,:,k)=transl(0, 0, humanoid_structure.waist.link_lengths(2))*waist_homogeneous_transform(:,:,k);
% 
%         %Arms pose
%         CoM_pose_RPY= pose_quat2rpy(h.LF_T_CoM(q(:,k)));
%         CoM_rotation = RPY2Rot_Mat(CoM_pose_RPY(4:6));
%         CoM_homogeneous_transform(:,:,k) = rt2tr(CoM_rotation, CoM_pose_RPY(1:3));    
% 
%         humanoid_right_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Leg(:,:,2);
%         humanoid_left_leg_base(:,:,k) = waist_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Leg(:,:,2);
% 
%         TEO_right_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Right_Arm(:,:,2);
%         TEO_left_arm_base(:,:,k) = CoM_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Left_Arm(:,:,2);
% 
%         TEO_torso_base(:,:,k) = torso_homogeneous_transform(:,:,k)*HT_Matrix_Plot_Torso(:,:,2);        
%     else
%         disp('ERROR!: Wrong trajectory Foot option') 
%     end
%         
%         
% end
%     
% 
% for k=1:size(q,2)
%     %Plot the number of q
%     if exist('q_text')
%         delete(q_text);
%     end;
%     
%     q_text = text(0,0.5,1.5,strcat('qt=',num2str(k)),'Color','g');
%     
%     %Plot the support phase
%     
%     
%     humanoid_right_leg.base = humanoid_right_leg_base(:,:,k);
%     humanoid_left_leg.base = humanoid_left_leg_base(:,:,k);
% 
%     TEO_right_arm.base = TEO_right_arm_base(:,:,k);
%     TEO_left_arm.base = TEO_left_arm_base(:,:,k);
% 
%     TEO_torso.base = TEO_torso_base(:,:,k);
%     
%     %%%Plot
%     %Legs
%     humanoid_right_leg.plot(q(1:6,k)'); 
%     humanoid_left_leg.plot(q(7:12,k)');   
%     
%     %Torso
%     TEO_torso.plot(q(13:14,k)'); 
%     
%     %Arms
%     TEO_right_arm.plot(q(15:20,k)');  
%     TEO_left_arm.plot(q(21:26,k)'); 
%     
%     drawnow
%     
% end