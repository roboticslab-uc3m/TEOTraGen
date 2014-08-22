function [ cog_traj, dcog_traj, ddcog_traj] = cog_3d_LIPM_trajectoryMODIFICADO(next_footprint, prev_footprint, Ts, step_times, beta, lambda, zc, g)
%COG_3D_LIPM Generates a Center of Gravity (CoG) using the 3D Linear
%Inverted Pendulum Model (LIPM)
%   Input:
%   Output:


cog_traj = [];
dcog_traj = [];
ddcog_traj = [];

pie = [];



prev_median_zmp = prev_footprint/2;
next_median_zmp = next_footprint/2;

% Change global reference to foot reference
DSpose1 = prev_median_zmp; % DSpose1(3) = zc;
DSpose2 = next_median_zmp; % DSpose2(3) = zc;

lambdaI = DSpose1(1)*lambda;
lambdaF = DSpose2(1)*lambda;

betaI = DSpose1(2)*beta;
%betaF = DSpose1(2)*beta;
betaF = DSpose2(2)*beta;

[cog_traj, dcog_traj, ddcog_traj] = com_traj_one_step(lambdaI, lambdaF, betaI, betaF, DSpose1, DSpose2, zc, g, Ts, step_times.Tinit, step_times.TDS1, step_times.TDS2, step_times.Tend);


% Le resto la posicion inicial de prev_footprint/2, para que comience en
% zeros(6,1)
cog_traj.data(1,:) = cog_traj.data(1,:) - prev_median_zmp(1);
cog_traj.data(2,:) = cog_traj.data(2,:) - prev_median_zmp(2);
cog_traj.data(3,:) = cog_traj.data(3,:) - prev_median_zmp(3);
cog_traj.data(4,:) = cog_traj.data(4,:) - prev_median_zmp(4);
cog_traj.data(5,:) = cog_traj.data(5,:) - prev_median_zmp(5);
cog_traj.data(6,:) = cog_traj.data(6,:) - prev_median_zmp(6);

end

function [cog_traj, dcog_traj, ddcog_traj] = com_traj_one_step(lambdaI, lambdaF, betaI, betaF, DSpose1, DSpose2, zc, g, Ts, Tinit, TDS1, TDS2, Tend)
% Simple Support Trajectory
% lambdaI
% lambdaF
% (tSS-tDS1)



%   step_times.Tinit = t0;
%   step_times.TDS1 = round_to_Ts(step_times.Tinit + TimeDS/2,Ts);
%   step_times.TSS = round_to_Ts(step_times.TDS1 + TimeSS/2,Ts);
%   step_times.TDS2 = round_to_Ts(step_times.TSS + TimeSS/2,Ts);
%   step_times.Tend = round_to_Ts(step_times.TDS2 + TimeDS/2,Ts);



[ppSS, dppSS] = cog_3d_LIPM(lambdaI, lambdaF, betaI, betaF, zc, g, round_to_Ts(TDS2-TDS1,Ts), Ts);

% figure(1),plot(ppSS(1,:))
% figure(2),plot(dppSS(1,:))


% pos_ppSS = [ppSS(1,:)+footprint_pose(1);ppSS(2,:)+footprint_pose(2);ppSS(3,:)+footprint_pose(3)];
CoGorientation = zeros(3,size(ppSS,2));
ppSS = [ppSS; CoGorientation];

% LE QUITO LA ALTURA
ppSS(3,:)=0;

% Convert foot reference to global reference
% ppSS = transform_frame(pose_rpy2tr(footprint_pose), pose_SS);

dppSS = [dppSS; zeros(3,size(dppSS,2))];
ddppSS = [zeros(6,1) diff(dppSS,1,2)];

tt = TDS1:Ts:TDS2;       
trajSS = create_trajectory_structure(ppSS, Ts, tt);
dtrajSS = create_trajectory_structure(dppSS, Ts, tt);
ddtrajSS = create_trajectory_structure(ddppSS, Ts, tt);

% First Double Support (Before Simple Support Trajectory)
Pds0 = set_trajectory_condition(Tinit, DSpose1, zeros(6,1), zeros(6,1));
Pds1 = set_trajectory_condition(TDS1, trajSS.data(:,1), dtrajSS.data(:,1), zeros(6,1));
PDS  = [Pds0 Pds1];
[trajDS1, dtrajDS1, ddtrajDS1] = poly5_segments_trajectory (PDS, Ts);
cog_traj = combine_2_traj(trajDS1, trajSS);
dcog_traj = combine_2_traj(dtrajDS1, dtrajSS);
ddcog_traj = combine_2_traj(ddtrajDS1, ddtrajSS);


% Second Double Support (After Simple Support Trajectory)
Pds0 = set_trajectory_condition(TDS2, trajSS.data(:,end), dtrajSS.data(:,end), zeros(6,1));
Pds1 = set_trajectory_condition(Tend, DSpose2, zeros(6,1), zeros(6,1));
PDS  = [Pds0 Pds1];
[trajDS2, dtrajDS2, ddtrajDS2] = poly5_segments_trajectory (PDS, Ts);
cog_traj = combine_2_traj(cog_traj, trajDS2);
dcog_traj = combine_2_traj(dcog_traj, dtrajDS2);
ddcog_traj = combine_2_traj(ddcog_traj, ddtrajDS2);

% figure(2),plot(cog_traj.time,cog_traj.data(1,:))
% figure(3),plot(cog_traj.time,cog_traj.data(2,:))
% figure(4),plot(cog_traj.data(1,:),cog_traj.data(2,:))

end

function combined_traj = combine_2_traj(traj1,traj2)
  combined_traj.data = [traj1.data(:,1:end-1) traj2.data(:,:)];
  combined_traj.Ts =  traj1.Ts;
  combined_traj.time = [traj1.time(:,1:end-1) traj2.time(:,:)];
  combined_traj.T = traj1.T + traj2.T;
end

function posesT = transform_frame(T, poses)
posesT = zeros(size(poses));
for ii=1:size(poses,2),
    posesT(:,ii) = pose_tr2rpy(T*pose_rpy2tr(poses(:,ii)));
end
end
