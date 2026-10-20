% startup.m
root = fileparts(mfilename("fullpath"));
p = strsplit(genpath(root), pathsep);
p = p(~contains(p, filesep + ["unused" ".simcache" "slprj" ".vscode"])); % parked files and build output stay off the path
addpath(strjoin(p, pathsep));
cache = fullfile(root, '.simcache'); % every slprj/ and .slxc lands here instead of wherever a sim was started
if ~strcmp(Simulink.fileGenControl('get', 'CacheFolder'), cache)
    Simulink.fileGenControl('set', 'CacheFolder', cache, 'CodeGenFolder', cache, 'createDir', true);
end
shareMATLABSession()
