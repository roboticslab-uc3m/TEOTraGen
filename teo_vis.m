function teo_vis ()
% Copyright (C) 2014 Universidad Carlos III de Madrid
% see the LICENSE file included with this software

% clear java;
% clear classes;

if (isdeployed)
    [path, folder, ~] = fileparts(ctfroot);
    root_path = fullfile(path, folder);
else
    root_path = fileparts(mfilename('fullpath'));
end
addpath(genpath(root_path));

app = gui.VisWindow(root_path, false);
app.launch_gui();

end