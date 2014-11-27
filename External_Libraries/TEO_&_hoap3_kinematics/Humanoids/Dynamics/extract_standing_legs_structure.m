function standing_legs_structure = extract_standing_legs_structure (legs, waist)
% EXTRACT_STANDING_LEGS_STRUCTURE extracts information from a structure of
% a humanoid robot and converts it in order to be used for the standing
% leg
%
%   See also EXTRACT_DYNAMIC_PARAMETERS.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/08/06 $

% Number of Joints
n = 6;

% Preallocate array of structures
right.link  = struct('mass', num2cell(zeros(1, n)), 'CoM', zeros(3,1), 'inertia', eye(3));
right.joint = struct('type', num2cell(zeros(1, n)));
left.link   = struct('mass', num2cell(zeros(1, n)), 'CoM', zeros(3,1), 'inertia', eye(3));
left.joint  = struct('type', num2cell(zeros(1, n)));

% Insert link masses and motor types which do not depend on the reference system
for jj = 1:n-1
    right.link(jj).mass  = legs.right.link(n - jj).mass;
    left.link(jj).mass   = legs.left.link(n - jj).mass;
end
jj = 6;
right.link(jj).mass  = waist.link(2).mass;
left.link(jj).mass   = waist.link(2).mass;

for jj = 1:n
    right.joint(jj).type = legs.right.joint(n - jj+1).type;
    left.joint(jj).type  = legs.left.joint(n - jj+1).type;
end

% Insert link CoMs and Inertias which do depend on the reference system
% Joint 1: Ankle Roll
jj = 1;
R1 = [-1, 0, 0; 0, 0, 1; 0, 1, 0];
right.link(jj).CoM = R1 * legs.right.link(n - jj).CoM;
left.link(jj).CoM  = R1 * legs.left.link(n - jj).CoM;
right.link(jj).inertia = rotate_inertia_matrix(legs.right.link(n - jj).inertia, R1);
left.link(jj).inertia  = rotate_inertia_matrix(legs.left.link(n - jj).inertia, R1);

% Joint 2: Ankle Pitch
jj = 2;
R2 = [-1, 0, 0; 0, 1, 0; 0, 0, -1];
right.link(jj).CoM = R2 * legs.right.link(n - jj).CoM + [legs.link_lengths(3); 0;  legs.link_lengths(4)];
left.link(jj).CoM  = R2 * legs.left.link(n - jj).CoM  + [legs.link_lengths(3); 0; -legs.link_lengths(4)];
right.link(jj).inertia = rotate_inertia_matrix(legs.right.link(n - jj).inertia, R2);
left.link(jj).inertia  = rotate_inertia_matrix(legs.left.link(n - jj).inertia, R2);

% Joint 3: Knee
jj = 3;
R3 = R2;
right.link(jj).CoM = R3 * legs.right.link(n - jj).CoM + [legs.link_lengths(2); 0; 0];
left.link(jj).CoM  = R3 * legs.left.link(n - jj).CoM  + [legs.link_lengths(2); 0; 0];
right.link(jj).inertia = rotate_inertia_matrix(legs.right.link(n - jj).inertia, R3);
left.link(jj).inertia  = rotate_inertia_matrix(legs.left.link(n - jj).inertia, R3);

% Joint 4: Hip Pitch
jj = 4;
R4 = [0, 1, 0; 0, 0, -1; -1, 0, 0];
right.link(jj).CoM = R4 * legs.right.link(n - jj).CoM;
left.link(jj).CoM  = R4 * legs.left.link(n - jj).CoM;
right.link(jj).inertia = rotate_inertia_matrix(legs.right.link(n - jj).inertia, R4);
left.link(jj).inertia  = rotate_inertia_matrix(legs.left.link(n - jj).inertia, R4);

% Joint 5: Hip Roll
jj = 5;
R5 = [0, 0, 1; 0, 1, 0; -1, 0, 0];
right.link(jj).CoM = R5 * legs.right.link(n - jj).CoM;
left.link(jj).CoM  = R5 * legs.left.link(n - jj).CoM;
right.link(jj).inertia = rotate_inertia_matrix(legs.right.link(n - jj).inertia, R5);
left.link(jj).inertia  = rotate_inertia_matrix(legs.left.link(n - jj).inertia, R5);

% Joint 5: Hip Yaw
jj = 6;
right.link(jj).CoM = waist.link(2).CoM + [ legs.link_lengths(1); 0; 0];
left.link(jj).CoM  = waist.link(2).CoM + [-legs.link_lengths(1); 0; 0];
right.link(jj).inertia = waist.link(2).inertia;
left.link(jj).inertia  = waist.link(2).inertia;

% Gravity
right.g0 = legs.right.g0;
left.g0 = legs.left.g0;

standing_legs_structure.right = right;
standing_legs_structure.left  = left;