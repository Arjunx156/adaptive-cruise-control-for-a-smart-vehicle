function dirPath = accResultsDir()
%ACCRESULTSDIR Absolute path to the results folder, created if missing.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
dirPath = fullfile(root, 'results');
if ~isfolder(dirPath)
    mkdir(dirPath);
end
end
