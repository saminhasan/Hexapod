function S = readSessionLog(file, cacheDir)
    % FEEDBACK stream + tx commands of one session log, parsed once then cached
    % S.fb: [t_ns slave uL yL vL uR yR vR playhead], angles deg body frame, v deg/s
    % S.tx: table(t, msg, value), STORE rows left out
    [~, nm] = fileparts(file);
    cacheFile = fullfile(cacheDir, nm + ".mat");
    if isfile(cacheFile)
        S = load(cacheFile);
        return
    end

    txt = fileread(file);
    num = '([-\d.eE+]+)';
    pat = ['"timestamp_ns": (\d+)[^\n]*?"feedback": \{"slave": (\d), "u_l": ' num ', "y_pos_l": ' num ', "y_vel_l": ' num ...
           ', "u_r": ' num ', "y_pos_r": ' num ', "y_vel_r": ' num ', "playhead": (\d+)\}'];
    tk = regexp(txt, pat, 'tokens');
    tk = [tk{:}];
    S.fb = reshape(sscanf(strjoin(tk, ' '), '%f'), 9, []).'; % sscanf, str2double is ~3x slower here

    tk = regexp(txt, '"kind": "tx", "recorded_ns": \d+, "data": \{"timestamp_ns": (\d+)[^\n]*?"msg": "(?!STORE")(\w+)"([^\n]*)', 'tokens');
    tk = vertcat(tk{:});
    t = str2double(tk(:,1));
    msg = string(tk(:,2));
    value = str2double(regexp(tk(:,3), '(?<="value": )[-\d.eE+]+', 'match', 'once'));
    S.tx = table(t, msg, value);

    if ~isfolder(cacheDir), mkdir(cacheDir); end
    save(cacheFile, '-struct', 'S');
end
