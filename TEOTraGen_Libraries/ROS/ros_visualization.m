function [ output ] = ros_visualization( q, SF, Ts )
%ROS_VISUALIZATION Summary of this function goes here
%   Detailed explanation goes here

%%%TODOOOOOOOOOOOOOOOOOOOOOOOOOO
% PONER EL VALOR DE LAS PARTES DE TEO, y su valor se obtiene titulos(jj)
% titulos = {'X' 'Y' 'Z' 'roll' 'pitch' 'yaw'};
% titulos(jj)
% TRUCO PARA CONCATENAR strcat('axes',char(axes_order(jj))

t = tcpip('localhost', 3552, 'NetworkRole', 'client');
disp('Esperando conexion con servidor')
fopen(t);


disp('Conexion establecida :D')
%Enviar los datos al servidor
pause(1)
disp('Enviando datos...')
for ndata = 1:size(q,2),
    pause(Ts);
%     Humanoid_joints_structure.joints = [q(1,ndata) q(7,ndata) q(2,ndata) q(8,ndata) q(3,ndata) q(9,ndata) q(4,ndata) q(10,ndata) q(5,ndata) q(11,ndata) q(6,ndata) q(12,ndata) ...
Humanoid_joints_structure.joints = [q(1,ndata) q(2,ndata) q(3,ndata) q(4,ndata) q(5,ndata) q(6,ndata) ...
                                    q(7,ndata) q(8,ndata) q(9,ndata) q(10,ndata) q(11,ndata) q(12,ndata) ...
                                    q(13,ndata) q(14,ndata) ... 
                                    q(15,ndata) q(16,ndata) q(17,ndata) q(18,ndata) q(19,ndata) q(20,ndata) ...
                                    q(21,ndata) q(22,ndata) q(23,ndata) q(24,ndata) q(25,ndata) q(26,ndata) ]; %...
%                                     q(13,ndata) q(14,ndata) ]; %...
% Humanoid_joints_structure.joints = [q(1,1) q(2,1) q(3,1) q(4,1) q(5,1) q(6,1) ...
%                                     q(7,1) q(8,1) q(9,1) q(10,1) q(11,1) q(12,1) ...
%                                     q(13,1) q(14,1) ]; %...

%                                     q(15,ndata) q(16,ndata) q(17,ndata) q(18,ndata) q(19,ndata) q(20,ndata) ...
%                                     q(21,ndata) q(22,ndata) q(23,ndata) q(24,ndata) q(25,ndata) q(26,ndata)];


% Double support is considered as Right Foot Support
if SF(ndata) == 0,
  SupportFoot = -1;
else
  SupportFoot = SF(ndata);
end
Humanoid_joints_structure.support_foot = SupportFoot;
    
% joint_names[0] = "right_hip_yaw";
% joint_names[1] = "left_hip_yaw";
% joint_names[2] = "right_hip_roll";
% joint_names[3] = "left_hip_roll";
% joint_names[4] = "right_hip_pitch";
% joint_names[5] = "left_hip_pitch";
% joint_names[6] = "right_knee_pitch";
% joint_names[7] = "left_knee_pitch";
% joint_names[8] = "right_ankle_pitch";
% joint_names[9] = "left_ankle_pitch";
% joint_names[10] = "right_ankle_roll";
% joint_names[11] = "left_ankle_roll";
% joint_names[12] = "l_shoulder_pitch";
% joint_names[13] = "l_shoulder_roll";
% joint_names[14] = "l_shoulder_yaw";
% joint_names[15] = "l_elbow_pitch";
% joint_names[16] = "l_wrist_yaw";
% joint_names[17] = "l_wrist_pitch";
% joint_names[18] = "r_shoulder_pitch";
% joint_names[19] = "r_shoulder_roll";
% joint_names[20] = "r_shoulder_yaw";
% joint_names[21] = "r_elbow_pitch";
% joint_names[22] = "r_wrist_yaw";
% joint_names[23] = "r_wrist_pitch";
    
    
%     dataenviar = [typecast(Humanoid_joints_structure.joints, 'int8')];
     dataenviar = [typecast(Humanoid_joints_structure.joints, 'int8') typecast(int32(Humanoid_joints_structure.support_foot), 'int8')];

%     dataenviar = [typecast(data.x, 'int8') int8(data.y)]; 
%     dataenviar = [data.x data.y];
    fwrite(t, dataenviar, 'int8');
%     str=['enviado: ',num2str(data.x)];
    str=['Enviado dato:', num2str(ndata), ' cantidad:' num2str(size(dataenviar))];
    disp(str);
%     data.x=data.x+1;
end

disp('Datos enviados');

fclose(t);

output = 1;

end

