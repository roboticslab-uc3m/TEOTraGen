function TEOTraGen_launch ()
% Copyright (C) 2014 Universidad Carlos III de Madrid
% see the LICENSE file included with this software

disp(' ')
disp('********************************************')
disp('**************** TEOTraGen *****************')
disp('********************************************')
disp(' ')
disp('TEO Humanoid Robot - Trajectory Generation')
disp(' ' )
disp('Robotics Lab, Universidad Carlos III de Madrid.')
disp('<a href = "http://roboticslab.uc3m.es">http://roboticslab.uc3m.es</a>')
disp(' ')
disp('Please wait some seconds...')
disp(' ')

if (isdeployed)
	[path, folder, ~] = fileparts(ctfroot);
	root_path = fullfile(path, folder);
else
	root_path = fileparts(mfilename('fullpath'));
end

addpath(genpath(root_path));

TEO_GUI = TEOTraGen();

end