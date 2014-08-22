clear all
clc
Ts = 1e-3;
N = 1001;

humanoid_fields = humanoid_operational_fields ();
trajectory = create_trajectory_template (humanoid_fields, Ts);

traj_rh2 = create_trajectory_structure(rand(6,N), Ts, 0:Ts:(N-1)*Ts);
trajectory = insert_trajectory(trajectory, humanoid_fields, traj_rh2, 'CoM');

traj_rh = create_trajectory_structure(rand(6,N), Ts, (0:Ts:(N-1)*Ts) + 2);
trajectory = insert_trajectory(trajectory, humanoid_fields, traj_rh, 'CoM');

traj_rh2 = create_trajectory_structure(rand(6,N), Ts, (0:Ts:(N-1)*Ts) + 2.5);
trajectory = insert_trajectory(trajectory, humanoid_fields, traj_rh2, 'CoM');