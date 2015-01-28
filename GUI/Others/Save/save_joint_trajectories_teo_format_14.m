function [ ] = save_joint_trajectories_teo_format_14(traj_name_string, q, dq, ddq)
%SAVE_TRAJECTORY_TEO_FORMAT_14 Summary of this function goes here
%   Detailed explanation goes here

if nargin == 0
  error('save_joints_trajectories_teo_format_14:argChk', 'Wrong number of input arguments')
end


%%%%%%%%%%%%%%%%%%%%%%%%
% Save joint positions %
%%%%%%%%%%%%%%%%%%%%%%%%

try
  [file1, path1] = uiputfile([traj_name_string '_q.csv'], 'Save Joint Angles as');
  % Convert to degrees
  q = radtodeg(q);

  % Change the signs of joints with different orientation
  q(6,:) = -q(6,:);
  q(7,:) = -q(7,:);
  q(8,:) = -q(8,:);
  q(15,:) = -q(15,:);
  q(17,:) = -q(17,:);
  q(18,:) = -q(18,:);
  q(19,:) = -q(19,:);
  q(20,:) = -q(20,:);
  q(23,:) = -q(23,:);
  q(25,:) = -q(25,:);

  csvid = fopen(strcat(path1,file1), 'w');
  fprintf(csvid, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n',...
                                                                                     [q(13,:); q(7,:); q(8,:); q(9,:); q(10,:); q(11,:); q(12,:); q(14,:); q(1,:); q(2,:); q(3,:); q(4,:); q(5,:); q(6,:)]);
  fclose(csvid);

catch
  disp('Save joint trajectories canceled');
end

    
%%%%%%%%%%%%%%%%%%%%%%%%%
% Save joint velocities %
%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin > 2
  try
    [file2, path2] = uiputfile([traj_name_string '_dq.csv'], 'Save Joint Velocities as');
  
    % Convert to degrees
    dq = radtodeg(dq);

    % Change the signs of joints with different orientation
    dq(6,:) = -dq(6,:);
    dq(7,:) = -dq(7,:);
    dq(8,:) = -dq(8,:);
    dq(15,:) = -dq(15,:);
    dq(17,:) = -dq(17,:);
    dq(18,:) = -dq(18,:);
    dq(19,:) = -dq(19,:);
    dq(20,:) = -dq(20,:);
    dq(23,:) = -dq(23,:);
    dq(25,:) = -dq(25,:);

    csvid = fopen(strcat(path2,file2), 'w');
    fprintf(csvid, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n',...
                                                                                       [dq(13,:); dq(7,:); dq(8,:); dq(9,:); dq(10,:); dq(11,:); dq(12,:); dq(14,:); dq(1,:); dq(2,:); dq(3,:); dq(4,:); dq(5,:); dq(6,:)]);
    fclose(csvid);

  catch
    disp('Save joint velocities canceled');
  end
  
end
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save joint accelerations %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
if nargin > 3
  try
    [file3, path3] = uiputfile([traj_name_string '_ddq.csv'], 'Save Joint Accelerations as');
    
    % Convert to degrees
    ddq = radtodeg(ddq);

    % Change the signs of joints with different orientation
    ddq(6,:) = -ddq(6,:);
    ddq(7,:) = -ddq(7,:);
    ddq(8,:) = -ddq(8,:);
    ddq(15,:) = -ddq(15,:);
    ddq(17,:) = -ddq(17,:);
    ddq(18,:) = -ddq(18,:);
    ddq(19,:) = -ddq(19,:);
    ddq(20,:) = -ddq(20,:);
    ddq(23,:) = -ddq(23,:);
    ddq(25,:) = -ddq(25,:);

    csvid = fopen(strcat(path3,file3), 'w');
    fprintf(csvid, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n',...
                                                                                       [ddq(13,:); ddq(7,:); ddq(8,:); ddq(9,:); ddq(10,:); ddq(11,:); ddq(12,:); ddq(14,:); ddq(1,:); ddq(2,:); ddq(3,:); ddq(4,:); ddq(5,:); ddq(6,:)]);
    fclose(csvid);
  catch
    disp('Save joint accelerations canceled');
  end
end

end

