function runs = runList()
    % runs that have a torso imu capture, X-1 left out
    % play = which PLAY of that session log the capture belongs to (matched off capture/PLAY times)
    name    = ["X-2"; "X-3"; "X-4"; "X-5"; "NOBRA-6"; "NOBRA-7"; "NOBRA-8"; "NOBRA-9"; "NOBRA-10"];
    imu     = ["TEST-BRARIG-34C-X-" + (2:5)'; "TEST-BRARIG-34D-NOBRA-" + (6:10)'];
    session = [repmat("session_20261005T163419", 4, 1); repmat("session_20261005T165558", 5, 1)];
    play    = [1 2 3 4 1 2 3 4 5]';
    cond    = [repmat("bra", 4, 1); repmat("nobra", 5, 1)];
    runs = table(name, imu, session, play, cond);
end
