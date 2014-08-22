function [trajectory d_trajectory dd_trajectory] = gait_operational_trajectory(traj, d_traj, dd_traj, q0, h)

% Author: Domingo Esteban

pose_RF_CoM = pose_quat2rpy(h.RF_T_CoM(q0));
pose_LF_CoM = pose_quat2rpy(h.LF_T_CoM(q0));
pose_w_RF = pose_quat2rpy(h.w_T_RF(q0));
pose_w_LF = pose_quat2rpy(h.w_T_LF(q0));
pose_CoM_RH = pose_quat2rpy(h.CoM_T_RH(q0));
pose_CoM_LH = pose_quat2rpy(h.CoM_T_LH(q0));

trajectory = traj;
d_trajectory = d_traj;
dd_trajectory = dd_traj;

Ts = traj.Ts;

if (traj.SF(1) == 0) || (traj.SF(1) == -1)
  trajectory.CoM(:,1) = pose_RF_CoM;
elseif (traj.SF(1) == 1)
  trajectory.CoM(:,1) = pose_LF_CoM;
else
  error('ErrorTEOTraGen:wrongOption', 'Wrong Support Foot option');
end

trajectory.RF(:,1) = pose_w_RF;
trajectory.LF(:,1) = pose_w_LF;
trajectory.RH(:,1) = pose_CoM_RH;
trajectory.LH(:,1) = pose_CoM_LH;

rCoMtrajectory = trajectory.CoM; rCoMtrajectory(:,1) = pose_RF_CoM;
lCoMtrajectory = trajectory.CoM; lCoMtrajectory(:,1) = pose_LF_CoM;

for ii=2:size(traj.CoM,2)
  
  rCoMtrajectory(:,ii) = rCoMtrajectory(:,ii-1) + d_trajectory.CoM(:,ii-1)*Ts;
  lCoMtrajectory(:,ii) = lCoMtrajectory(:,ii-1) + d_trajectory.CoM(:,ii-1)*Ts;
  trajectory.CoM(:,ii) = trajectory.CoM(:,ii-1) + d_trajectory.CoM(:,ii-1)*Ts;
  
  trajectory.RH(:,ii) = trajectory.RH(:,ii-1) + d_trajectory.RH(:,ii-1)*Ts;
  trajectory.LH(:,ii) = trajectory.LH(:,ii-1) + d_trajectory.LH(:,ii-1)*Ts;
  
  d_trajectory.RH(:,ii) = d_trajectory.RH(:,ii-1);
  d_trajectory.LH(:,ii) = d_trajectory.LH(:,ii-1);
  dd_trajectory.RH(:,ii) = dd_trajectory.RH(:,ii-1);
  dd_trajectory.LH(:,ii) = dd_trajectory.LH(:,ii-1);
  
%   trajectory.RF(:,ii) = trajectory.RF(:,ii-1) + d_trajectory.RF(:,ii-1)*Ts;
%   trajectory.LF(:,ii) = trajectory.LF(:,ii-1) + d_trajectory.LF(:,ii-1)*Ts;
  
  if traj.SF(ii) == 0, 
%     trajectory.CoM(:,ii) = rCoMtrajectory(:,ii);
    % D.E.: Ver si lo anterior solo se resta hoy a que transformar la orientacion
  elseif traj.SF(ii) == -1,  
%     trajectory.CoM(:,ii) = rCoMtrajectory(:,ii);
%     trajectory.RF(:,ii) = trajectory.RF(:,ii-1) - d_trajectory.CoM(:,ii-1)*Ts;
%     trajectory.LF(:,ii) = trajectory.LF(:,ii-1) + d_trajectory.LF(:,ii-1)*Ts ;
  elseif traj.SF(ii) == 1,  
%     trajectory.CoM(:,ii) = lCoMtrajectory(:,ii);
%     trajectory.RF(:,ii) = trajectory.RF(:,ii-1) + d_trajectory.RF(:,ii-1)*Ts;
%     trajectory.LF(:,ii) = trajectory.LF(:,ii-1) - d_trajectory.CoM(:,ii-1)*Ts;
  else
    error('ErrorTEOTraGen:wrongOption', 'Wrong Support Foot option'); 
  end
  
   trajectory.RF(:,ii) = trajectory.RF(:,ii-1) + d_trajectory.RF(:,ii-1)*Ts;
  if traj.SF(ii) == 0, 
  trajectory.LF(:,ii) = trajectory.LF(:,ii-1) + d_trajectory.LF(:,ii-1)*Ts - d_trajectory.CoM(:,ii-1)*Ts;
  else
  trajectory.LF(:,ii) = trajectory.LF(:,ii-1) + d_trajectory.LF(:,ii-1)*Ts;
  end;
  
  d_trajectory.RF(:,ii) = d_trajectory.RF(:,ii) - d_trajectory.CoM(:,ii);
  d_trajectory.LF(:,ii) = d_trajectory.LF(:,ii) - d_trajectory.CoM(:,ii);
  dd_trajectory.RF(:,ii) = dd_trajectory.RF(:,ii) - dd_trajectory.CoM(:,ii);
  dd_trajectory.LF(:,ii) = dd_trajectory.LF(:,ii) - dd_trajectory.CoM(:,ii);

%   if traj.SF(ii) == 0, 
%     trajectory.CoM(:,ii) = rCoMtrajectory(:,ii);
%     trajectory.RF(:,ii) = trajectory.RF(:,ii-1) - d_trajectory.CoM(:,ii-1)*Ts;
%     trajectory.LF(:,ii) = trajectory.LF(:,ii-1) - d_trajectory.CoM(:,ii-1)*Ts;
%     % D.E.: Ver si lo anterior solo se resta hoy a que transformar la orientacion
%   elseif traj.SF(ii) == -1,  
%     trajectory.CoM(:,ii) = rCoMtrajectory(:,ii);
%     trajectory.RF(:,ii) = trajectory.RF(:,ii-1) - d_trajectory.CoM(:,ii-1)*Ts;
%     trajectory.LF(:,ii) = trajectory.LF(:,ii-1) + d_trajectory.LF(:,ii-1)*Ts ;
%   elseif traj.SF(ii) == 1,  
%     trajectory.CoM(:,ii) = lCoMtrajectory(:,ii);
%     trajectory.RF(:,ii) = trajectory.RF(:,ii-1) + d_trajectory.RF(:,ii-1)*Ts;
%     trajectory.LF(:,ii) = trajectory.LF(:,ii-1) - d_trajectory.CoM(:,ii-1)*Ts;
%   else
%     error('ErrorTEOTraGen:wrongOption', 'Wrong Support Foot option'); 
%   end
%   if traj.SF(ii) == 0,
%     trajectory.RF(:,ii) = trajectory.RF(:,ii-1) + d_trajectory.RF(:,ii-1)*Ts;
%     trajectory.LF(:,ii) = trajectory.LF(:,ii-1) - d_trajectory.CoM(:,ii-1)*Ts;
%     % D.E.: Ver si lo anterior solo se resta hoy a que transformar la orientacion
%   else
%     trajectory.RF(:,ii) = trajectory.RF(:,ii-1) + d_trajectory.RF(:,ii-1)*Ts;
%     trajectory.LF(:,ii) = trajectory.LF(:,ii-1) + d_trajectory.LF(:,ii-1)*Ts;
%   end
end