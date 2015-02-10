clc

disp('TEO Humanoid Robot - Test Real Trajectory')
disp('Robotics Lab, Universidad Carlos III de Madrid.')
disp('<a href = "http://roboticslab.uc3m.es">http://roboticslab.uc3m.es</a>')
disp(' ')

global q dq ddq trajectory d_trajectory dd_trajectory data

subindex = @(A,r,c) A(r,c);


h = TEO_kinematics_library();
humanoid_fields = humanoid_operational_fields (); 
humanoid_description = TEO_structure('numeric', 'rad', 'm');


planned_trajectory_global = create_trajectory_template (humanoid_fields, data.Ts);
d_planned_trajectory_global = create_trajectory_template (humanoid_fields, data.Ts);
dd_planned_trajectory_global = create_trajectory_template (humanoid_fields, data.Ts);
real_trajectory_global = create_trajectory_template (humanoid_fields, data.Ts);
d_real_trajectory_global = create_trajectory_template (humanoid_fields, data.Ts);
dd_real_trajectory_global = create_trajectory_template (humanoid_fields, data.Ts);



L = length(trajectory.time);
LF_p0_w = pose_quat2rpy(real(h.LF_T_w(q(:,1))));
RF_p0_w = pose_quat2rpy(real(h.RF_T_w(q(:,1))));
LF_p0_CoM = pose_quat2rpy(real(h.LF_T_CoM(q(:,1))));
RF_p0_CoM = pose_quat2rpy(real(h.RF_T_CoM(q(:,1))));
w_p0_RF = pose_quat2rpy(real(h.w_T_RF(q(:,1))));
w_p0_LF = pose_quat2rpy(real(h.w_T_LF(q(:,1))));
CoM_p0_RF = pose_quat2rpy(real(h.CoM_T_RF(q(:,1))));
CoM_p0_LF = pose_quat2rpy(real(h.CoM_T_LF(q(:,1))));
CoM_p0_RH = pose_quat2rpy(real(h.CoM_T_RH(q(:,1))));
CoM_p0_LH = pose_quat2rpy(real(h.CoM_T_LH(q(:,1))));

world_T0_CoM = [0; 0; (subindex(pose_quat2rpy(real(h.RF_T_CoM(q(:,1)))),3,1) - subindex(pose_quat2rpy(real(h.CoM_T_RF(q(:,1)))),3,1))/2; 0; 0; 0];
world_T0_RF = [0; (subindex(-pose_quat2rpy(real(h.RF_T_CoM(q(:,1)))),2,1) + subindex(pose_quat2rpy(real(h.CoM_T_RF(q(:,1)))),2,1))/2; 0; 0; 0; 0];
world_T0_LF = [0; (subindex(-pose_quat2rpy(real(h.LF_T_CoM(q(:,1)))),2,1) + subindex(pose_quat2rpy(real(h.CoM_T_LF(q(:,1)))),2,1))/2; 0; 0; 0; 0];


% First Poses
planned_trajectory_global.CoM = world_T0_CoM;
planned_trajectory_global.RF = world_T0_RF;
planned_trajectory_global.LF = world_T0_LF;
real_trajectory_global.CoM = world_T0_CoM;
real_trajectory_global.RF = world_T0_RF;
real_trajectory_global.LF = world_T0_LF;


% Planned Trajectories

for jj = 2:L,  
  planned_trajectory_global.CoM(:,jj) = trajectory.CoM(:,jj) + world_T0_CoM;
  
  switch trajectory.SF(jj),
    case 0
      planned_trajectory_global.RF(:,jj) = planned_trajectory_global.RF(:,jj-1);
      planned_trajectory_global.LF(:,jj) = planned_trajectory_global.LF(:,jj-1);
    case -1
      if (trajectory.SF(jj-1) == 0)
        world_T0_RF = planned_trajectory_global.RF(:,jj-1);
      end
%       planned_trajectory_global.RF(:,jj) = trajectory.RF(:,jj) + world_T0_RF;
%       planned_trajectory_global.LF(:,jj) = (planned_trajectory_global.CoM(:,jj)-planned_trajectory_global.CoM(:,jj-1)) + trajectory.LF(:,jj) + world_T0_LF;
      planned_trajectory_global.RF(:,jj) = (trajectory.RF(:,jj) - trajectory.RF(:,jj-1)) + planned_trajectory_global.RF(:,jj-1);
      planned_trajectory_global.LF(:,jj) = (planned_trajectory_global.CoM(:,jj) - planned_trajectory_global.CoM(:,jj-1)) + (trajectory.LF(:,jj) - trajectory.LF(:,jj-1)) + planned_trajectory_global.LF(:,jj-1);
    case 1
      if (trajectory.SF(jj-1) == 0)
        world_T0_LF = planned_trajectory_global.LF(:,jj-1);
      end
%       planned_trajectory_global.RF(:,jj) = (planned_trajectory_global.CoM(:,jj)-planned_trajectory_global.CoM(:,jj-1)) + trajectory.RF(:,jj) + world_T0_RF;
%       planned_trajectory_global.LF(:,jj) = trajectory.LF(:,jj) + world_T0_LF;
      planned_trajectory_global.RF(:,jj) = (planned_trajectory_global.CoM(:,jj) - planned_trajectory_global.CoM(:,jj-1)) + (trajectory.RF(:,jj) - trajectory.RF(:,jj-1)) + planned_trajectory_global.RF(:,jj-1);
      planned_trajectory_global.LF(:,jj) = (trajectory.LF(:,jj) - trajectory.LF(:,jj-1)) + planned_trajectory_global.LF(:,jj-1);
  end
  

end

planned_trajectory_global.LH = zeros(size(trajectory.LH));
planned_trajectory_global.RH = zeros(size(trajectory.RH));

planned_trajectory_global.SF = trajectory.SF;
planned_trajectory_global.time = trajectory.time;
planned_trajectory_global.T = trajectory.T;

for jj = 1:(length(humanoid_fields)-1),
  d_planned_trajectory_global.(humanoid_fields(jj).name) = [diff(planned_trajectory_global.(humanoid_fields(jj).name),1,2) zeros(size(planned_trajectory_global.(humanoid_fields(jj).name),1),1)]*(1/data.Ts);
  dd_planned_trajectory_global.(humanoid_fields(jj).name) = [diff(d_planned_trajectory_global.(humanoid_fields(jj).name),1,2) zeros(size(d_planned_trajectory_global.(humanoid_fields(jj).name),1),1)]*(1/data.Ts);
end


% Real Trajectories

for jj = 2:L,
  switch trajectory.SF(jj),
    case 0
      real_trajectory_global.RF(:,jj) = real_trajectory_global.RF(:,jj-1);
      real_trajectory_global.LF(:,jj) = real_trajectory_global.LF(:,jj-1);
    case -1
      if (trajectory.SF(jj-1) == 0)
        world_T0_RF = real_trajectory_global.RF(:,jj-1);
      end
      real_trajectory_global.RF(:,jj) = real_trajectory_global.RF(:,jj-1);
      delta_com = pose_quat2rpy(real(h.RF_T_CoM(q(:,jj)))) - pose_quat2rpy(real(h.RF_T_CoM(q(:,jj-1))));
      delta_lf = pose_quat2rpy(real(h.CoM_T_LF(q(:,jj)))) - pose_quat2rpy(real(h.CoM_T_LF(q(:,jj-1))));
      real_trajectory_global.CoM(:,jj) = delta_com + real_trajectory_global.CoM(:,jj-1);
      real_trajectory_global.LF(:,jj) = delta_com + delta_lf + real_trajectory_global.LF(:,jj-1);
    case 1
      if (trajectory.SF(jj-1) == 0)
        world_T0_LF = real_trajectory_global.LF(:,jj-1);
      end
      real_trajectory_global.LF(:,jj) = real_trajectory_global.LF(:,jj-1);
      delta_com = pose_quat2rpy(real(h.LF_T_CoM(q(:,jj)))) - pose_quat2rpy(real(h.LF_T_CoM(q(:,jj-1))));
      delta_rf = pose_quat2rpy(real(h.CoM_T_RF(q(:,jj)))) - pose_quat2rpy(real(h.CoM_T_RF(q(:,jj-1))));
      real_trajectory_global.CoM(:,jj) = delta_com + real_trajectory_global.CoM(:,jj-1);
      real_trajectory_global.RF(:,jj) = delta_com + delta_rf + real_trajectory_global.RF(:,jj-1);
  end
  
	real_trajectory_global.CoM(:,jj) = trajectory.CoM(:,jj) + world_T0_CoM;
end

real_trajectory_global.LH = zeros(size(trajectory.LH));
real_trajectory_global.RH = zeros(size(trajectory.RH));

real_trajectory_global.SF = trajectory.SF;
real_trajectory_global.time = trajectory.time;
real_trajectory_global.T = trajectory.T;

for jj = 1:(length(humanoid_fields)-1),
  d_real_trajectory_global.(humanoid_fields(jj).name) = [diff(real_trajectory_global.(humanoid_fields(jj).name),1,2) zeros(size(real_trajectory_global.(humanoid_fields(jj).name),1),1)]*(1/data.Ts);
  dd_real_trajectory_global.(humanoid_fields(jj).name) = [diff(d_real_trajectory_global.(humanoid_fields(jj).name),1,2) zeros(size(d_real_trajectory_global.(humanoid_fields(jj).name),1),1)]*(1/data.Ts);
end


% figure(88)
% plot(trajectory.time, planned_trajectory_global.RF(1,:), trajectory.time, trajectory.SF(:)*0.2);
% plot(planned_trajectory_global.RF(1,:), planned_trajectory_global.RF(3,:));
% plot(planned_trajectory_global.LF(1,:), planned_trajectory_global.LF(3,:));
% plot(trajectory.time, real_trajectory_global.RF(1,:), trajectory.time, trajectory.SF(:)*0.2);
% plot(real_trajectory_global.RF(1,:), real_trajectory_global.RF(3,:));
% plot(real_trajectory_global.LF(1,:), real_trajectory_global.LF(3,:));

% 3D Plot World
% x_resolution = 0.01;
% y_resolution = 0.01;
% z_resolution = 0.01;
% plot_world = figure;
% figure(plot_world)
% hold on
%   plot_com = plot3(planned_trajectory_global.CoM(1,:), planned_trajectory_global.CoM(2,:), planned_trajectory_global.CoM(3,:));
%   plot_rf = plot3(planned_trajectory_global.RF(1,:), planned_trajectory_global.RF(2,:), planned_trajectory_global.RF(3,:), 'color', 'red');
%   plot_lf = plot3(planned_trajectory_global.LF(1,:), planned_trajectory_global.LF(2,:), planned_trajectory_global.LF(3,:), 'color', 'black');
%   % set(gca,'XTick',min(planned_trajectory_global.CoM(1,:)):x_resolution:max(planned_trajectory_global.CoM(1,:)), 'XMinorTick','on')
% hold off
% hold on
%   plot_com = plot3(real_trajectory_global.CoM(1,:), real_trajectory_global.CoM(2,:), real_trajectory_global.CoM(3,:));
%   plot_rf = plot3(real_trajectory_global.RF(1,:), real_trajectory_global.RF(2,:), real_trajectory_global.RF(3,:), 'color', 'red');
%   plot_lf = plot3(real_trajectory_global.LF(1,:), real_trajectory_global.LF(2,:), real_trajectory_global.LF(3,:), 'color', 'black');
%   % set(gca,'XTick',min(planned_trajectory_global.CoM(1,:)):x_resolution:max(planned_trajectory_global.CoM(1,:)), 'XMinorTick','on')
% hold off



% figure(90), plot(planned_trajectory_global.time, planned_trajectory_global.RF(3,:),real_trajectory_global.time, real_trajectory_global.RF(3,:)); xlabel('time'); ylabel('z'); legend('Planned Trajectory','Real Trajectory'); title('Trajectory RF - Z axis');
% figure(91), plot(planned_trajectory_global.time, planned_trajectory_global.LF(3,:),real_trajectory_global.time, real_trajectory_global.LF(3,:)); xlabel('time'); ylabel('z'); legend('Planned Trajectory','Real Trajectory'); title('Trajectory LF - Z axis');
% figure(92), plot(planned_trajectory_global.time, planned_trajectory_global.CoM(1,:),real_trajectory_global.time, real_trajectory_global.CoM(1,:)); xlabel('time'); ylabel('x'); legend('Planned Trajectory','Real Trajectory'); title('Trajectory CoM - X axis');
% figure(93), plot(planned_trajectory_global.time, planned_trajectory_global.CoM(2,:),real_trajectory_global.time, real_trajectory_global.CoM(2,:)); xlabel('time'); ylabel('y'); legend('Planned Trajectory','Real Trajectory'); title('Trajectory CoM - Y axis');

planned_data.trajectory = planned_trajectory_global;
planned_data.d_trajectory = d_planned_trajectory_global;
planned_data.dd_trajectory = dd_planned_trajectory_global;
real_data.trajectory = real_trajectory_global;
real_data.d_trajectory = d_real_trajectory_global;
real_data.dd_trajectory = dd_real_trajectory_global;

trajectory_planned_real_plots(planned_data, real_data, humanoid_description);