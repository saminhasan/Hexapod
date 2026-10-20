function file = exportActuatorAngles(ref, name)
    % writes ref.q for the rig: actuatorAngles/<name>.csv, what the firmware loads (6 crank angles in rad, one row
    % per 1 ms tick, no header), plus <name>.json next to it describing the file. name defaults to <mode>_<cycles>c,
    % e.g. syn_30c, or the mode when there are no cycles, e.g. imuThrow34D. A file already there with the same
    % content is left alone; one with different content is reported and kept (file comes back empty)
    if nargin < 2 || isempty(name)
        if isempty(ref.cycles), name = ref.mode; else, name = sprintf('%s_%dc', ref.mode, ref.cycles); end
    end
    folder = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'actuatorAngles');
    file = fullfile(folder, [name '.csv']);
    tmp = [tempname '.csv'];
    writematrix(ref.q, tmp);
    hash = sha256(tmp);
    if isfile(file)
        same = strcmp(hash, sha256(file));
        delete(tmp);
        if same
            fprintf('%s is already up to date\n', file);
        else
            fprintf('%s already exists with different content, kept it; pass another name to write this one\n', file);
            file = '';
        end
        return
    end
    movefile(tmp, file);

    meta.file = [name '.csv'];
    meta.created = char(datetime('now', 'Format', 'yyyy-MM-dd''T''HH:mm:ss'));
    meta.mode = ref.mode;
    meta.cycles = ref.cycles;
    meta.rows = size(ref.q, 1);
    meta.rateHz = round(1/(ref.t(2) - ref.t(1)));
    meta.durationS = ref.t(end) - ref.t(1);
    meta.columns = 'q1..q6 crank angles (rad), inverseKinematics order';
    meta.rampTimeS = ref.rampTime;
    meta.bodyToPlatformM = ref.bodyToPlatform(:).';
    meta.centerOffsetM = ref.centerOffset(:).';
    meta.sha256 = hash;
    writelines(jsonencode(meta, 'PrettyPrint', true), fullfile(folder, [name '.json']));
    fprintf('wrote %s\n', file);
end
