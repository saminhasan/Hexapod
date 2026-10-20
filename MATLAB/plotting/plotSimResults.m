function plotSimResults(simOut, ref, robot)
    % platform figures then motor figures for one sim run against its reference
    plotPlatformInfo(simOut, ref, robot);
    plotMotorInfo(simOut, ref, robot);
end
