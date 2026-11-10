function setupPaths()
%SETUPPATHS Add all project folders to the MATLAB path.
    root = fileparts(mfilename('fullpath'));
    folders = {'utils', 'robot', 'environment', 'rrt', 'state_machine', 'simulink'};
    for k = 1:numel(folders)
        f = fullfile(root, folders{k});
        if isfolder(f)
            addpath(f);
        end
    end
    addpath(root);
end
