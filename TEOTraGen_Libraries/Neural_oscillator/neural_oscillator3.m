%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  “smooth4” oscillator  pattern %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parameters
Vmax = 5;
Vmin = -5;
tolerance = 0.1;
increment = 0.1;
gE = 0.8; % gain Exciter state
gI = 0.8; % gain Inhibitor state

we2i = 1;
wi2e = 1;

ndata = 20;

oscillator_data = zeros(1, ndata);


%  Initialize variables.
j = 1;                        % j is the time variable (x-axis)
V_stable = (Vmax + Vmin)/2.0; % V_stable is set as the midpoint between Vmax and Vmin
Vm = V_stable;                % Vm is the output of the system
Vd_work = Vmax;               % The current max value for Vm
V0_work = Vmin; % the current min value for Vm
state = 'e'; % state “e” is for exciter, i for inhibitor
T = tolerance; % Set T to the tolerence value, we want to reach Vd_work and V0_work within tolerance value T.
C = increment; % Set C to the value that Vm is increment/decremented by

oscillator_data(1) = V_stable;

% Generate patterns.
while (j <= ndata),
	% Excite state.
  % In the excite state, Vm is incremented until it reaches Vmax
  if strcmp(state,'e')
    Ve = Vmax - Vm; % Exciter state error
    Vc = gE * Ve;
    Vm = Vm + Vc;
    % If exceeds the current max value for Vm, Vd_work, transiton to the inhibit state.
    if (Vm >= ( Vd_work-T))
      % Set the state to inhibit.
      state = 'i';
      % Diminish the current min value for Vm, V0_work, by the weight from Exciter to Inhibitor, we2i.
      V0_work = V0_work * we2i;
    end

  % Inhibit state.
	elseif strcmp(state,'i')
    Ve = Vmin - Vm; % Inhibitor state error
    Vc = gI * Ve;
    Vm = Vm + Vc;
    % If Vm is less than the current min value for Vm, V0_work, transiton to the excite state.
    if ( Vm <= ( V0_work + T))
      % Set the state to excite.
      state = 'e';
      % Increase the current max value for Vm, Vd_work, by the weight from Inhibitor to Excitor, wi2e.
      Vd_work = Vd_work * wi2e;
   end
  end
  
  % Output data.
	oscillator_data(j+1) =  Vm;
  
  % Increment count.
  j = j + 1;
    
end

plot(oscillator_data);