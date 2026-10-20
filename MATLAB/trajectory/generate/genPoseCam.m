function trajectory = genPoseCam(n)
    % camera-recorded running motion, z and rotations (x, y stay 0), repeated n times and splined to 1 kHz:
    % [t x y z Rx Ry Rz]. trajectory/data/Rots100.csv is the filtered capture, Rots_raw.csv the raw one
    dt = 1e-3;
    file = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data', 'Rots100.csv');
    data = table2array(readtable(file, 'CommentStyle', '#'));
    ts = mean(diff(data(:,1)));
    tEnd = (data(end,1) + ts)*n - ts;
    raw = zeros(height(data)*n, 7);
    raw(:,1) = (0:ts:tEnd)';
    raw(:,4:7) = repmat(data(:,4:7) - mean(data(:,4:7), 1), n, 1);
    raw(:,5:7) = deg2rad(raw(:,5:7));
    raw(:,6) = raw(:,6) - mean(raw(:,6));
    t = (0:dt:tEnd)';
    trajectory = [t, spline(raw(:,1), raw(:,2:7)', t')'];
end
