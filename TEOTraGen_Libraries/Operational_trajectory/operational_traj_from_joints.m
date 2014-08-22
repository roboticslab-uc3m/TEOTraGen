function [ trajectory, d_trajectory, dd_trajectory] = operational_traj_from_joints( q, dq, ddq, Ts, h, support_foot, time)
%OPERATIONAL_TRAJ_FROM_JOINTS Summary of this function goes here
%   Detailed explanation goes here
% Input:
% - Ts: Sample Time
humanoid_fields = humanoid_operational_fields (); 

trajectory = create_trajectory_template (humanoid_fields, Ts);
d_trajectory = create_trajectory_template (humanoid_fields, Ts);
dd_trajectory = create_trajectory_template (humanoid_fields, Ts);
% 
% pose_quat2rpy(real(h.LF_T_w(q)))
% 
% 

if ~isnumeric(support_foot),
  switch support_foot
    case 'Double Support'
      support_foot = 0;
    case 'Right Foot Support'
      support_foot = 1;
    case 'Left Foot Support'
      support_foot = -1;
    otherwise
      error('ErrorTEOTraGen:wrongInputGUI', 'Wrong support_foot value');
  end
else
  if ~((support_foot == 0) || (support_foot == 1) || (support_foot == -1))
    error('ErrorTEOTraGen:wrongInputGUI', 'Wrong support_foot value');
  end
end

switch support_foot
  case 0     % double support
    com_traj = pose_quat2rpy(real(h.RF_T_CoM(q)));
    rf_traj = zeros(size(com_traj));
    lf_traj = zeros(size(com_traj));
  case 1     % left foot support
    com_traj = pose_quat2rpy(real(h.LF_T_CoM(q)));
    rf_traj = pose_quat2rpy(real(h.CoM_T_RF(q)));
    lf_traj = zeros(size(com_traj));
  case -1     % right foot support
    com_traj = pose_quat2rpy(real(h.RF_T_CoM(q)));
    rf_traj = zeros(size(com_traj));
    lf_traj = pose_quat2rpy(real(h.CoM_T_LF(q)));
end

rh_traj = pose_quat2rpy(real(h.CoM_T_RH(q)));
lh_traj = pose_quat2rpy(real(h.CoM_T_LH(q)));

% Convert to trajectory structure
com_traj_struct = create_trajectory_structure(com_traj, Ts, time);
rf_traj_struct = create_trajectory_structure(rf_traj, Ts, time);
lf_traj_struct = create_trajectory_structure(lf_traj, Ts, time);
rh_traj_struct = create_trajectory_structure(rh_traj, Ts, time);
lh_traj_struct = create_trajectory_structure(lh_traj, Ts, time);

% Insert to operational trajectory structure
trajectory   = insert_trajectory(trajectory, humanoid_fields, com_traj_struct, 'CoM');
trajectory   = insert_trajectory(trajectory, humanoid_fields, rf_traj_struct, 'RF');
trajectory   = insert_trajectory(trajectory, humanoid_fields, lf_traj_struct, 'LF');
trajectory   = insert_trajectory(trajectory, humanoid_fields, rh_traj_struct, 'RH');
trajectory   = insert_trajectory(trajectory, humanoid_fields, lh_traj_struct, 'LH');

% Velocity
com_d_traj = [zeros(6,1) diff(com_traj,1,2)/Ts];
rf_d_traj = [zeros(6,1) diff(rf_traj,1,2)/Ts];
lf_d_traj = [zeros(6,1) diff(lf_traj,1,2)/Ts];
rh_d_traj = [zeros(6,1) diff(rh_traj,1,2)/Ts];
lh_d_traj = [zeros(6,1) diff(lh_traj,1,2)/Ts];

d_trajectory   = insert_trajectory(d_trajectory, humanoid_fields, create_trajectory_structure(com_d_traj, Ts, time), 'CoM');
d_trajectory   = insert_trajectory(d_trajectory, humanoid_fields, create_trajectory_structure(rf_d_traj, Ts, time), 'RF');
d_trajectory   = insert_trajectory(d_trajectory, humanoid_fields, create_trajectory_structure(lf_d_traj, Ts, time), 'LF');
d_trajectory   = insert_trajectory(d_trajectory, humanoid_fields, create_trajectory_structure(rh_d_traj, Ts, time), 'RH');
d_trajectory   = insert_trajectory(d_trajectory, humanoid_fields, create_trajectory_structure(lh_d_traj, Ts, time), 'LH');

% Acceleration
com_dd_traj = [zeros(6,1) diff(com_d_traj,1,2)/Ts];
rf_dd_traj = [zeros(6,1) diff(rf_d_traj,1,2)/Ts];
lf_dd_traj = [zeros(6,1) diff(lf_d_traj,1,2)/Ts];
rh_dd_traj = [zeros(6,1) diff(rh_d_traj,1,2)/Ts];
lh_dd_traj = [zeros(6,1) diff(lh_d_traj,1,2)/Ts];

dd_trajectory   = insert_trajectory(dd_trajectory, humanoid_fields, create_trajectory_structure(com_dd_traj, Ts, time), 'CoM');
dd_trajectory   = insert_trajectory(dd_trajectory, humanoid_fields, create_trajectory_structure(rf_dd_traj, Ts, time), 'RF');
dd_trajectory   = insert_trajectory(dd_trajectory, humanoid_fields, create_trajectory_structure(lf_dd_traj, Ts, time), 'LF');
dd_trajectory   = insert_trajectory(dd_trajectory, humanoid_fields, create_trajectory_structure(rh_dd_traj, Ts, time), 'RH');
dd_trajectory   = insert_trajectory(dd_trajectory, humanoid_fields, create_trajectory_structure(lh_dd_traj, Ts, time), 'LH');

% 
% 
% values = ones(1,size(traj_stretch.time,2))*trajectory_points.support_foot.(SETTINGS_TEO.humanoid_fields(jj).name)(ii);
% support_foot_stretch = create_trajectory_structure(values,   trajectory_points.Ts_val.(SETTINGS_TEO.humanoid_fields(jj).name)(ii), traj_stretch.time);
% trajectory   = insert_trajectory(trajectory, humanoid_fields, support_foot_stretch ,  'SF');
% d_trajectory = insert_trajectory(d_trajectory, humanoid_fields, support_foot_stretch , 'SF');
% dd_trajectory = insert_trajectory(dd_trajectory, humanoid_fields, support_foot_stretch , 'SF');

end

