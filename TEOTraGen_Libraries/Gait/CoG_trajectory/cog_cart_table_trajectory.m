%% Center of Gravity trajectory using *Cart Table* model

function [cog_traj, dcog_traj, ddcog_traj] = cog_cart_table_trajectory(algorithm, desired_zmp, Ts, step_times, zc, beta, lambda, g, zmptime)
%COG_CART_TABLE_TRAJECTORY Generates the Center of Gravity (CoG) trajectory
%using the Cart Table model
%   Input:
%   Output:

global zmpPRUEBA

if ~strcmp(algorithm,'Kajita') && ~strcmp(algorithm,'Wieber')
  disp('ERROR: Opcion no implementada');
  
else
  
  %%%%%%%%%%%%%%
  % Parameters %
  %%%%%%%%%%%%%%
  
  T = Ts;
  
  N = 300;
  RQratio = 1e-05;
  
  
  % Add N ZMP values at the end
  desired_zmp = [desired_zmp repmat(desired_zmp(:,end),1,N)];
  
  
  K = length(desired_zmp);
  
  ZMPxref = desired_zmp(1,:)';
  ZMPyref = desired_zmp(2,:)';
  
  cog_traj = zeros(6,K-N);
  cog_traj(3,:) = ones(1,K-N)*zc;
  dcog_traj = zeros(6,K-N);
  ddcog_traj = zeros(6,K-N);
    
  %%%%%%%%%%
  % ZMP real %
  %%%%%%%%%%
  zmpk = zeros(3,K);
  
  if strcmp(algorithm,'Kajita')
    
    %%%%%%%%%%%%%%%%%%%%
    % Program solution %
    %%%%%%%%%%%%%%%%%%%%
    xk = zeros(3,K); % State vector of the CoM
    yk = zeros(3,K); % State vector of the CoM
       
    %%%%%%%%%%%%%%%%%%% 
    % System matrixes %
    %%%%%%%%%%%%%%%%%%%
    A = [1   T  T^2/2;
         0   1   T;
         0   0   1];

    B = [T^3/6; T^2/2; T];

    C = [1 0 -zc/g];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % optimal gain refered to preview H2 theory %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    A1 = [1 C*A;zeros(3,1) A];
    B1 = [C*B;B];
    C1 = [1 0 0 0];

    Qe = 1;
    Qx = zeros(3,3);

    Q = [Qe zeros(1,3);zeros(3,1) Qx];
    R = 1e-6;

    % P = solution of DARE, G = optimal gain
    [P,L,G] = dare(A1,B1,Q,R);

    % Optimal gain from DARE
    Ke = G(1,1);
    Kx = G(1,2:end);


    Ac = A1-B1*G;
    Gp = zeros(1,N); % Preview gain matrix
    X1 = zeros(4,N); % Preview state

    Gp(1) = -Ke;
    X1(:,1) = -Ac'*P*[1;0;0;0];

    for i = 2:N
      Gp(i) = (R + B1'*P*B1)\B1'*X1(:,i-1);
      X1(:,i) = Ac'*X1(:,i-1);
    end

    
    eX = 0; % summation of error
    eY = 0; % summation of error
    
    
    % Preallocation
    u = zeros(2,K); % 
    
    
    for ii = 1:K; 
   
      % Px matrix
       
      if (ii + N) < K
        zmpk(1,ii) = C*xk(:,ii);
        zmpk(2,ii) = C*yk(:,ii);
        
        eX = eX + zmpk(1,ii) - ZMPxref(ii);
        eY = eY + zmpk(2,ii) - ZMPyref(ii);
        prevX = 0;
        prevY = 0;
        
        for kk = 1:N
          if (ii+N)<K
            prevX = prevX + 1*Gp(kk)*ZMPxref(ii+kk);
            prevY = prevY + 1*Gp(kk)*ZMPyref(ii+kk);
          end
        end

      else    
        zmpk(1,ii) = zmpk(1,ii-1);
        zmpk(2,ii) = zmpk(2,ii-1);   
        
      end

      
      u(1,ii) = -Ke*eX - Kx*xk(:,ii) - prevX;
      u(2,ii) = -Ke*eY - Kx*yk(:,ii) - prevY;
      
      if ii < K        
        xk(:,ii+1) = A*xk(:,ii) + B*u(1,ii);
        yk(:,ii+1) = A*yk(:,ii) + B*u(2,ii);
      end

    end
    
    

  elseif  strcmp(algorithm,'Wieber')

    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Algorithm preallocation %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    Px = zeros(N,3);
    Pu = zeros(N,N);
    PuValues = zeros(N,1); % Necessary to generate Pu

    Inxn = eye(N);
    u = zeros(2,K); % 
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Quadratic Program solution %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xk = zeros(3,K); % State vector of the CoM
    yk = zeros(3,K); % State vector of the CoM
       
    %%%%%%%%%%%%%%%%%%% 
    % System matrixes %
    %%%%%%%%%%%%%%%%%%%
    A = [1   T  T^2/2;
         0   1   T;
         0   0   1];

    B = [T^3/6; T^2/2; T];

    C = [1 0 -zc/g];

    %%%%%%%%%%%%%%
    % Generation %
    %%%%%%%%%%%%%%


    for jj = 1:N,
      Px(jj,:) = [1 jj*T jj^2*T^2/2-zc/g];
      PuValues(jj,1) = (1+3*(jj-1)+3*(jj-1)^2)*T^3/6-T*zc/g;
    end   

    % Pu matrix
    Pu = tril(toeplitz(PuValues));

    MultiplyMatrix = (Pu'*Pu + RQratio*Inxn)\Pu';
    
    
    for ii = 1:K; 
   
      % Px matrix
       zmpk(1,ii) = C*xk(:,ii);
       zmpk(2,ii) = C*yk(:,ii);

      if (ii + N) < K
    %     % Completo
    %     Xk = - MatrizX*(Px*xk(:,ii)-Zref(ii:ii+N-1,1));
    %     u(:,ii) = e'*Xk;

        % Solo el dato que importa
        u(1,ii) = - MultiplyMatrix(1,:)*(Px*xk(:,ii)-ZMPxref(ii:ii+N-1,1));
        u(2,ii) = - MultiplyMatrix(1,:)*(Px*yk(:,ii)-ZMPyref(ii:ii+N-1,1));

      else    
        u(1,ii) = u(1,ii-1);
        u(2,ii) = u(2,ii-1);

%         u(1,ii) = - MultiplyMatrix(1,:)*(Px*xk(:,ii)-ZMPxref(ii:ii+N-1,1));
%         u(2,ii) = - MultiplyMatrix(1,:)*(Px*yk(:,ii)-ZMPyref(ii:ii+N-1,1));
        
      end

      if ii < K        
        xk(:,ii+1) = A*xk(:,ii) + B*u(1,ii);
        yk(:,ii+1) = A*yk(:,ii) + B*u(2,ii);
        
        zmpPRUEBA.x(:,ii) = C*xk(:,ii);
        zmpPRUEBA.y(:,ii) = C*yk(:,ii);
      end

    end
    

  else
    disp('AHHHHHHHHHHHHHHHH');
  end  
  
   
%   cog_traj(1,:) = xk(1,:);
%   cog_traj(2,:) = yk(1,:);
%   
%   dcog_traj(1,:) = xk(2,:);
%   dcog_traj(2,:) = yk(2,:);  
%   
%   ddcog_traj(1,:) = xk(3,:);
%   ddcog_traj(2,:) = yk(3,:);  
  
  
  % Remove the added N values  
  cog_traj(1,:) = xk(1,1:end-N);
  cog_traj(2,:) = yk(1,1:end-N);
  
  dcog_traj(1,:) = xk(2,1:end-N);
  dcog_traj(2,:) = yk(2,1:end-N);  
  
  ddcog_traj(1,:) = xk(3,1:end-N);
  ddcog_traj(2,:) = yk(3,1:end-N); 
  
  
  
  cog_traj = create_trajectory_structure(cog_traj, Ts, zmptime);
  dcog_traj = create_trajectory_structure(dcog_traj, Ts, zmptime);
  ddcog_traj = create_trajectory_structure(ddcog_traj, Ts, zmptime);
  
end


end

