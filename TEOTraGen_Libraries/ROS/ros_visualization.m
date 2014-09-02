function [ output ] = ros_visualization( q, SF, Ts )
%ROS_VISUALIZATION Summary of this function goes here
%   Detailed explanation goes here


t = tcpip('localhost', 3552, 'NetworkRole', 'client');
disp('Esperando conexion con servidor')
fopen(t);


% disp('Conexion establecida :D')

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


% Double support is considered as Right Foot Support
if SF(ndata) == 0,
  SupportFoot = -1;
else
  SupportFoot = SF(ndata);
end

Humanoid_joints_structure.support_foot = SupportFoot;


	dataenviar = [typecast(Humanoid_joints_structure.joints, 'int8') typecast(int32(Humanoid_joints_structure.support_foot), 'int8')];

  fwrite(t, dataenviar, 'int8');

%   str=['Enviado dato:', num2str(ndata), ' cantidad:' num2str(size(dataenviar))];
%   disp(str);

end

% disp('Datos enviados');

fclose(t);

output = 1;

end

