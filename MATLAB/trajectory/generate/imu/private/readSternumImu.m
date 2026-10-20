function imu = readSternumImu(folder)
    % sternum IMU of one recording in oct 6/ExemplarBreastTorsoMovements_IMUs/<folder>: t (s), f specific force
    % (m/s^2), w rate (rad/s), sensor axes (x up, y side, z forward, see IMU_legend.xlsx). The csv is read once
    % and cached in tempdir (outside OneDrive)
    root = fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))));
    data = fullfile(root, 'oct 6', 'ExemplarBreastTorsoMovements_IMUs', 'ExemplarBreastTorsoMovements_IMUs', folder);
    csv = dir(fullfile(data, '*STERNUM*.csv'));
    assert(isscalar(csv), 'readSternumImu:File', 'expected one *STERNUM*.csv in %s', data);
    cache = fullfile(tempdir, 'hexapodImu', [folder '.mat']);
    if isfile(cache) && dir(cache).datenum > csv.datenum
        load(cache, 'M');
    else
        M = readmatrix(fullfile(data, csv.name), 'NumHeaderLines', 1);
        if ~isfolder(fileparts(cache)), mkdir(fileparts(cache)); end
        save(cache, 'M');
    end
    imu.t = M(:,1);
    imu.f = M(:,2:4);
    imu.w = deg2rad(M(:,5:7));
end
